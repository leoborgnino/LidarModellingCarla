import glob
import os
import sys
import argparse
import time
from datetime import datetime
import random
import numpy as np
from numpy import random
from matplotlib import cm
import open3d as o3d
from queue import Queue
from queue import Empty
import logging
import cv2
import math
import json

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from transformation_utils import build_projection_matrix, w3D_to_cam3D, w3D_to_cam2D, bbox_in_image
from kitti_label import KittiLabel

""" OUTPUT FOLDERS """
OUTPUT_FOLDER = "training"
POINTCLOUDS_FOLDER = "velodyne"

LIST_VEHICLES_PATH = "../Unreal/CarlaUE4/LidarModelFiles/vehicles.json"


def sensor_callback(data,queue):
    queue.put(data)


def create_output_folders():
    current_datetime = datetime.now().strftime("%d-%m-%y_%X")
    output_directory = os.path.join(current_datetime,OUTPUT_FOLDER)

    pointcloud_path = create_folder(output_directory, POINTCLOUDS_FOLDER)

    return pointcloud_path

def create_folder(output_directory,folder):
    output_folder = os.path.join(output_directory,folder)
    os.makedirs(output_folder)

    return output_folder

def generate_lidar_bp(blueprint_library,delta):
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('upper_fov', str(2.0))
    lidar_bp.set_attribute('lower_fov', str(-24.8))
    lidar_bp.set_attribute('channels', str(64))
    lidar_bp.set_attribute('range', str(120))
    lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
    lidar_bp.set_attribute('points_per_second', str(1300000))
    lidar_bp.set_attribute('noise_stddev', str(0.01))
    lidar_bp.set_attribute('dropoff_general_rate',str(0.0))
    lidar_bp.set_attribute('dropoff_intensity_limit',str(0.0))
    lidar_bp.set_attribute('dropoff_zero_intensity',str(0.0))
    #lidar_bp.set_attribute('sensor_tick', str(0.1)) #si es el doble q delta, va a dar un dato cada 2 ticks
    return lidar_bp

def spawn_vehicles(spawn_points,number_of_vehicles,vehicles_bp,traffic_manager,client):
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor
    vehicles_list = []

    batch = []
    for n, transform in enumerate(spawn_points):
        if n >= number_of_vehicles:
            break
        blueprint = random.choice(vehicles_bp)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        blueprint.set_attribute('role_name', 'autopilot')

        batch.append(SpawnActor(blueprint, transform)
            .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

    for response in client.apply_batch_sync(batch, True):
        if response.error:
            logging.error(response.error)
        else:
            vehicles_list.append(response.actor_id)

    return vehicles_list

def save_pointcloud(pointclouds_path,pointcloud):
    pc = np.copy(np.frombuffer(pointcloud.raw_data, dtype=np.dtype('f4')))
    pc = np.reshape(pc, (int(pc.shape[0] / 4), 4))
    pointcloud_path = './%s/%.6d.bin' % (pointclouds_path, pointcloud.frame) 
    pc.tofile(pointcloud_path)
    #print('point cloud %.6d.bin guardada' % lidar_data.frame)


def load_list_of_vehicles():
    file = open(LIST_VEHICLES_PATH)
    data=json.load(file)
    file.close()

    list_of_vehicles=[]
    for vehicle_id in data['vehicles']:
        list_of_vehicles.append(vehicle_id['api_bp_name'])

    return list_of_vehicles

def main(arg):
    """Spawnea el LIDAR HDL-64E y una camara RGB en un vehiculo, y genera un paso de simulacion"""
    #Cliente y simulador
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    
    #Crear directorios para guardar nubes de puntos
    pointclouds_path = create_output_folders()

    try:
        #Setea modo sincrono en la simulacion con delta fijo
        original_settings = world.get_settings()
        settings = world.get_settings()

        #traffic manager
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5) #distancia a mantener entre vehiculos
        #traffic_manager.set_hybrid_physics_mode(True) #desactiva las fisicas de los vehiculos lejanos al vehiculo hero, reduce el computo
        #traffic_manager.set_hybrid_physics_radius(100.0) #dentro de este radio, si se calculan las fisicas
        
        delta = 0.05
        #delta = 0.1
        settings.fixed_delta_seconds = delta
        settings.synchronous_mode = True
        world.apply_settings(settings)
        
        blueprint_library = world.get_blueprint_library()

        #Crea el LIDAR con las especificaciones de HDL-64E
        lidar_bp = generate_lidar_bp(blueprint_library,delta)

        #Crea vehiculo sobre el cual montar los sensores
        vehicle_bp = blueprint_library.filter("vehicle.lincoln.mkz_2017")[0]
        #vehicle_bp.set_attribute('role_name', 'hero')
        vehicle_bp.set_attribute('role_name', 'autopilot')

        #Spawnear vehiculo y sensores
        sp_vehicle = 0

        vehicle_transform = world.get_map().get_spawn_points()[sp_vehicle]
        vehicle = world.spawn_actor(
            blueprint=vehicle_bp,
            transform=vehicle_transform)
        vehicle.set_autopilot(True)

        #las coordenadas son relativas al vehiculo
        #x es el eje correspondiente a la direccion del auto, positivo seria hacia adelante
        #z es la altura

        lidar = world.spawn_actor(
            blueprint=lidar_bp,
            transform=carla.Transform(carla.Location(x=-0.27, z=1.73)), #posicion segun Kitti
            attach_to=vehicle)

        #Funciones de callback para almacenar nube de puntos
        lidar_queue= Queue()

        #Al recibir un nuevo data, se almacena en la cola
        lidar.listen(lambda data: sensor_callback(data,lidar_queue))
        
        #spawnear trafico 
        list_of_vehicles = load_list_of_vehicles() #cargar los vehiculos a spawnear

        vehicles_bp = blueprint_library.filter('vehicle.*') #blueprints de todos los vehiculos
        vehicles_bp = [x for x in vehicles_bp if x.id.endswith(tuple(list_of_vehicles)) ]
        #print(vehicles_bp)
        
        vehicles_bp = sorted(vehicles_bp, key=lambda bp: bp.id)
        spawn_points = world.get_map().get_spawn_points() #puntos de spawn del mapa en uso
        spawn_points.pop(sp_vehicle) #se quita el punto en el que se spawnea el vehivulo con los sensores
        random.shuffle(spawn_points) #mezclar 

        number_of_vehicles = 30

        vehicles_list = spawn_vehicles(spawn_points, number_of_vehicles, vehicles_bp, traffic_manager,client)

        #generar los ticks de simulacion necesarios hasta capturar los frames necesarios

        frames = arg.frames
        frames_captured = 0
        ticks = 0

        while True:

            world.tick()
            ticks += 1
            time.sleep(0.1)

            if not lidar_queue.empty():
                pointcloud = lidar_queue.get()

            save_pointcloud(pointclouds_path,pointcloud)

            frames_captured += 1

            sys.stdout.write("\r Capturados %d frames" % (frames_captured) + ' ')
            sys.stdout.flush()



    finally:
        world.apply_settings(original_settings)
        vehicle.destroy()
        lidar.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        time.sleep(0.5)

if __name__ == "__main__":
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '-f', '--frames',
        default=10,
        type=int,
        help='cantidad de frames a capturar, default: 10')
    args = argparser.parse_args()

    try:
        main(args)
    except KeyboardInterrupt:
        print(' - Exited by user.')
