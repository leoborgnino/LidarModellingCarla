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

from transformation_utils import build_projection_matrix, w3D_to_cam3D, w3D_to_cam2D, bbox_in_image,w3D_to_lidar3D, save_calibration_matrices, is_in_image, is_in_front
from carla_utils import generate_lidar_bp,generate_camera_bp, spawn_vehicles, save_image, save_pointcloud, sensor_callback, spawn_pedestrians

from kitti_label import KittiLabel, generate_kittilabel

""" OUTPUT FOLDERS """
OUTPUT_FOLDER = "training"
IMAGES_FOLDER = "image_2"
POINTCLOUDS_FOLDER = "velodyne"
LABELS_FOLDER = "label_2"
CALIB_FOLDER = "calib"

LIST_VEHICLES_PATH = "../../Unreal/CarlaUE4/LidarModelFiles/vehicles.json"

""" SAVE PATHS """
LIDAR_PATH = os.path.join(OUTPUT_FOLDER, 'velodyne/{0:06}.bin')
LABEL_PATH = os.path.join(OUTPUT_FOLDER, 'label_2/{0:06}.txt')
IMAGE_PATH = os.path.join(OUTPUT_FOLDER, 'image_2/{0:06}.png')
CALIBRATION_PATH = os.path.join(OUTPUT_FOLDER, 'calib/{0:06}.txt')

def create_folder(output_directory,folder):
    output_folder = os.path.join(output_directory,folder)
    os.makedirs(output_folder)

    return output_folder

def create_output_folders():
    current_datetime = datetime.now().strftime("%d-%m-%y_%X")
    output_directory = os.path.join(current_datetime,OUTPUT_FOLDER)

    images_path = create_folder(output_directory, IMAGES_FOLDER)
    pointcloud_path = create_folder(output_directory, POINTCLOUDS_FOLDER)
    calib_path = create_folder(output_directory, CALIB_FOLDER)
    labels_path = create_folder(output_directory, LABELS_FOLDER)

    return images_path,pointcloud_path,calib_path,labels_path

def load_list_of_vehicles():
    file = open(LIST_VEHICLES_PATH)
    data=json.load(file)
    file.close()

    list_of_cars=[]
    list_of_bikes=[]

    for vehicle_id in data['vehicles']:
        if(vehicle_id['api_bp_name'].endswith(BIKE_BP_NAME)):
            list_of_bikes.append(vehicle_id['api_bp_name'])
        else:
            list_of_cars.append(vehicle_id['api_bp_name'])

    return list_of_cars,list_of_bikes

BIKE_BP_NAME = 'diamondback.century'

def main(arg):
    """Spawnea el LIDAR HDL-64E y una camara RGB en un vehiculo, y genera un paso de simulacion"""
    #Cliente y simulador
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    
    #Crear directorios para guardar los datos, nubes de puntos, imagenes y labels
    images_path,pointclouds_path,calib_path,labels_path = create_output_folders()

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
        traffic_manager.global_percentage_speed_difference(80)
        #delta = 0.05
        delta = 0.1 #Determina los hz del sensor, 0.1 serian 10hz
        settings.fixed_delta_seconds = delta
        settings.synchronous_mode = True
        world.apply_settings(settings)
        
        blueprint_library = world.get_blueprint_library()

        #Crea el LIDAR con las especificaciones de HDL-64E
        ang_inc = True
        material = True
        reflectance_limit = False
        lidar_bp = generate_lidar_bp(blueprint_library,delta,ang_inc,material,reflectance_limit)

        #Crea la camara RGB
        camera_bp = generate_camera_bp(blueprint_library)

        #Crea vehiculo sobre el cual montar los sensores
        vehicle_bp = blueprint_library.filter("vehicle.seat.leon")[0]
        #vehicle_bp.set_attribute('role_name', 'hero')
        vehicle_bp.set_attribute('role_name', 'autopilot')

        #Spawnear vehiculo y sensores
        sp_vehicle = 0

        vehicle_transform = world.get_map().get_spawn_points()[sp_vehicle]
        vehicle = world.spawn_actor(
            blueprint=vehicle_bp,
            transform=vehicle_transform)
        vehicle.set_autopilot(True,traffic_manager.get_port())

        #las coordenadas son relativas al vehiculo
        #x es el eje correspondiente a la direccion del auto, positivo seria hacia adelante
        #z es la altura
        camera = world.spawn_actor(blueprint=camera_bp,
            transform=carla.Transform(carla.Location(x=0.0,y=-0.06, z=1.65)), #posicion segun Kitti 
            attach_to=vehicle)

        lidar = world.spawn_actor(
            blueprint=lidar_bp,
            transform=carla.Transform(carla.Location(x=-0.27, z=1.73)), #posicion segun Kitti
            attach_to=vehicle)
        #CAMBIOS FAVORABLES - UBICACION LIDAR Y VELOCIDAD REDUCIDA
        #Funciones de callback para almacenar imagen y nube de puntos
        image_queue= Queue()
        lidar_queue= Queue()

        #Al recibir un nuevo data, se almacena en la cola
        lidar.listen(lambda data: sensor_callback(data,lidar_queue))
        camera.listen(lambda data: sensor_callback(data,image_queue))
        
        #spawnear trafico 
        list_of_cars,list_of_bikes = load_list_of_vehicles()

        cars_bp = blueprint_library.filter('vehicle.*') #blueprints de todos los vehiculos
        cars_bp = [x for x in cars_bp if x.id.endswith(tuple(list_of_cars)) ]

        bikes_bp = blueprint_library.filter('vehicle.*') #blueprints de todos los vehiculos
        bikes_bp = [x for x in bikes_bp if x.id.endswith(tuple(list_of_bikes)) ]

        #print(vehicles_bp)
        
        cars_bp = sorted(cars_bp, key=lambda bp: bp.id)
        spawn_points = world.get_map().get_spawn_points() #puntos de spawn del mapa en uso
        spawn_points.pop(sp_vehicle) #se quita el punto en el que se spawnea el vehivulo con los sensores
        print(len(spawn_points))
        random.shuffle(spawn_points) #mezclar 

        cars_spawn_points = spawn_points[:int(len(spawn_points)/2)]
        bikes_spawn_points = spawn_points[int(len(spawn_points)/2):]

        number_of_cars = 20
        cars_list = spawn_vehicles(cars_spawn_points, number_of_cars, cars_bp, traffic_manager,client)

        number_of_bikes = 20
        bikes_list = spawn_vehicles(bikes_spawn_points, number_of_bikes, bikes_bp, traffic_manager,client)
        
        pedestrians_bp = blueprint_library.filter('walker.pedestrian.*')
        pedestrians_bp = [x for x in pedestrians_bp if int(x.get_attribute('generation')) == 2]

        number_of_walkers = 30
        all_actors, all_id = spawn_pedestrians(0,world,client,number_of_walkers,pedestrians_bp)

        #generar los ticks de simulacion necesarios hasta capturar los frames necesarios
        frames = arg.frames
        frames_captured = 0
        ticks = 0
        frames_between_captures = 0

        # Calculate the camera projection matrix to project from 3D -> 2D
        image_w = camera_bp.get_attribute("image_size_x").as_int()
        image_h = camera_bp.get_attribute("image_size_y").as_int()
        fov = camera_bp.get_attribute("fov").as_float()
        K = build_projection_matrix(image_w, image_h, fov)

        #Matriz de rotacion entre coordenadas de lidar y coordenadas de camara    
        rot_matrix = np.array([[0, -1, 0],
                                [0, 0, -1],
                                [1, 0,  0]])

        print("Inicio de captura")
        while frames_captured < frames:

            world.tick()
            ticks += 1
            time.sleep(0.1)

            #tras cada tick, en caso de haber dato disponible, se lo obtiene
            if not image_queue.empty():
                image = image_queue.get()
                while(image.frame != world.get_snapshot().frame):
                    image = image_queue.get()
                
            if not lidar_queue.empty():
                pointcloud = lidar_queue.get()
                while(pointcloud.frame != world.get_snapshot().frame):
                    pointcloud = lidar_queue.get()
            
            frames_between_captures += 1

            #print(image.frame)
            #print(pointcloud.frame)
            #print(world.get_snapshot().frame)
            #print("--------------------------------")
            #cuando se tengan ambos datos (imagen y nube de puntos) de un mismo frame, se almacenan ambos
            if ticks > 1 and (image.frame == pointcloud.frame):

                frames_between_captures = 0

                frame = frames_captured
                frames_captured = frames_captured + 1 

                sys.stdout.write("\r Capturados %d frames de %d" % (frames_captured,frames) + ' ')
                sys.stdout.flush()

                #guardar imagen
                save_image(images_path,image,frame)

                #guardar nube de puntos
                save_pointcloud(pointclouds_path,pointcloud,frame)

                #crear archivo de label
                label_file_name = './%s/%.6d.txt' % (labels_path, frame)
                label_file = open(label_file_name,'w')

                # Posicion de la camara para transformar coordenadas
                world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

                #posicion del lidar para transformar coordenadas
                lidar_2_world = np.array(lidar.get_transform().get_matrix())
                world_2_lidar = np.array(lidar.get_transform().get_inverse_matrix())

                #Matriz Transformacion de lidar a cam, solo traslacion sin rotacion de ejes
                lidar_2_cam = np.dot(world_2_camera,lidar_2_world)

                #Vector de traslacion de lidar a cam
                translation_vector = np.array([lidar_2_cam[0,3],lidar_2_cam[1,3],lidar_2_cam[2,3]])
                
                #Matriz de transformacion, traslacion + rotacion
                rot_trans_matrix = np.column_stack((rot_matrix,translation_vector))

                calibration_file_name = './%s/%.6d.txt' % (calib_path,frame)
                save_calibration_matrices(calibration_file_name,K,rot_trans_matrix)

                #Obtener todos los actores "vehiculos" y para cada uno
                for npc in world.get_actors().filter('*vehicle*'):
                    #print(npc.type_id)
                    if npc.id != vehicle.id:
                        #se determina si el npc se encuentra en frente del vehiculo
                        npc_is_in_front = is_in_front(vehicle,npc)

                        #se calcula la distancia entre los vehiculos
                        dist = npc.get_transform().location.distance(vehicle.get_transform().location)

                        #Solo se computan las bounding boxes de actores en frente y que esten a menos de una cierta distancia
                        if npc_is_in_front and dist < 130:

                            #se determina si el centro del vehiculo aparece dentro de los limites de la imagen
                            npc_is_in_image = is_in_image(npc,K,world_2_camera,image_w,image_h)

                            if(npc_is_in_image):
                                #Se crea el label de este npc y se obtiene la informacion necesaria
                                if(npc.type_id.endswith('diamondback.century') ): #bicicleta
                                    label = generate_kittilabel('Cyclist',npc, vehicle, world_2_camera,K,image_w,image_h,world_2_lidar,rot_trans_matrix)
                                else:
                                    label = generate_kittilabel('Car',npc, vehicle, world_2_camera,K,image_w,image_h,world_2_lidar,rot_trans_matrix)

                                str_label = label.get_label()
                                label_file.write("{}\n".format(str_label))

                for npc in world.get_actors().filter('*pedestrian*'):
                    #print("peaton detectado")
                    #se determina si el npc se encuentra en frente del vehiculo
                    npc_is_in_front = is_in_front(vehicle,npc)

                    #se calcula la distancia entre los vehiculos
                    dist = npc.get_transform().location.distance(vehicle.get_transform().location)

                    #Solo se computan las bounding boxes de actores en frente y que esten a menos de una cierta distancia
                    if npc_is_in_front and dist < 130:

                        #se determina si el centro del vehiculo aparece dentro de los limites de la imagen
                        npc_is_in_image = is_in_image(npc,K,world_2_camera,image_w,image_h)

                        if(npc_is_in_image):
                            
                            #Se crea el label de este npc y se obtiene la informacion necesaria
                            label = generate_kittilabel('Pedestrian',npc, vehicle, world_2_camera,K,image_w,image_h,world_2_lidar,rot_trans_matrix)
                            str_label = label.get_label()
                            label_file.write("{}\n".format(str_label))
                
                label_file.close()

    finally:
        world.apply_settings(original_settings)
        vehicle.destroy()
        lidar.destroy()
        camera.destroy()
        cv2.destroyAllWindows()

        client.apply_batch([carla.command.DestroyActor(x) for x in cars_list])
        client.apply_batch([carla.command.DestroyActor(x) for x in bikes_list])


        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])
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
