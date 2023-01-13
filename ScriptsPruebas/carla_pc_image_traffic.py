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

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

def sensor_callback(data,queue):
    queue.put(data)

def lidar_callback(point_cloud):
    """Recibe la nube de puntos desde el simulador y la guarda en formato binario"""

    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    data = np.reshape(data, (int(data.shape[0] / 4), 4))

    data.tofile('./point_clouds/%.6d.bin' % point_cloud.frame)
    print('point cloud %.6d.bin guardada' % point_cloud.frame)

def camera_callback(image):
    """Recibe la imagen desde el simulador y la guarda en formato png"""

    image.save_to_disk('./images/%6d.png' % image.frame)
    print('imagen %.6d.bin guardada' % image.frame)

def main(arg):
    """Spawnea el LIDAR HDL-64E y una camara RGB en un vehiculo, y genera un paso de simulacion"""
    #Cliente y simulador
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    
    #Crear directorios para guardar los datos, nubes de puntos e imagenes
    current_datetime = datetime.now().strftime("%d-%m-%y_%X")
    images_folder = 'images/' + current_datetime
    pointclouds_folder = 'point_clouds/' + current_datetime
    os.makedirs(images_folder)
    os.makedirs(pointclouds_folder)

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
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('upper_fov', str(2.0))
        lidar_bp.set_attribute('lower_fov', str(-24.8))
        lidar_bp.set_attribute('channels', str(64))
        lidar_bp.set_attribute('range', str(120))
        lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
        lidar_bp.set_attribute('points_per_second', str(1300000))
        #lidar_bp.set_attribute('points_per_second', str(500000))
        lidar_bp.set_attribute('noise_stddev', str(0.01))
        lidar_bp.set_attribute('dropoff_general_rate',str(0.0))
        #lidar_bp.set_attribute('sensor_tick', str(0.1)) #si es el doble q delta, va a dar un dato cada 2 ticks

        #Crea la camara RGB
        camera_bp = blueprint_library.filter("sensor.camera.rgb")[0]
        camera_bp.set_attribute("image_size_x", str(1242)) #Resolucion de las imagenes de Kitti
        camera_bp.set_attribute("image_size_y", str(375))

        #Crea vehiculo sobre el cual montar los sensores
        vehicle_bp = blueprint_library.filter("vehicle.lincoln.mkz_2017")[0]
        #vehicle_bp.set_attribute('role_name', 'hero')
        vehicle_bp.set_attribute('role_name', 'autopilot')

        #Spawnear vehiculo y sensores
        sp_vehicle_sensors = 0
        vehicle_transform = world.get_map().get_spawn_points()[sp_vehicle_sensors]
        vehicle = world.spawn_actor(
            blueprint=vehicle_bp,
            transform=vehicle_transform)
        vehicle.set_autopilot(True)

        camera = world.spawn_actor(
            blueprint=camera_bp,
            transform=carla.Transform(carla.Location(x=0.5, z=1.6)), #posicion segun Kitti
            attach_to=vehicle)

        lidar = world.spawn_actor(
            blueprint=lidar_bp,
            transform=carla.Transform(carla.Location(x=-0.5, z=1.8)), #posicion segun Kitti
            attach_to=vehicle)

        #Funciones de callback para almacenar imagen y nube de puntos
        image_queue= Queue()
        lidar_queue= Queue()

        #Al recibir un nuevo data, se almacena en la cola
        lidar.listen(lambda data: sensor_callback(data,lidar_queue))
        camera.listen(lambda data: sensor_callback(data,image_queue))
        
        #spawnear trafico 
        list_of_vehicles = ['audi.tt','citroen.c3','mini.cooper_s','chevrolet.impala']
        
        vehicles_bp = blueprint_library.filter('vehicle.*') #blueprints de todos los vehiculos
        vehicles_bp = [x for x in vehicles_bp if \
                        x.id.endswith(list_of_vehicles[0]) or \
                        x.id.endswith(list_of_vehicles[1]) or \
                        x.id.endswith(list_of_vehicles[2]) or \
                        x.id.endswith(list_of_vehicles[3])]
        vehicles_bp = sorted(vehicles_bp, key=lambda bp: bp.id)
        spawn_points = world.get_map().get_spawn_points() #puntos de spawn del mapa en uso
        spawn_points.pop(sp_vehicle_sensors) #se quita el punto en el que se spawnea el vehivulo con los sensores
        random.shuffle(spawn_points) #mezclar 

        number_of_vehicles = 30
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

        #generar los ticks de simulacion necesarios hasta capturar los frames necesarios

        frames = arg.frames
        frames_captured = 0
        ticks = 0
        
        while frames_captured < frames:

            world.tick()
            ticks += 1
            #time.sleep(0.005)

            #tras cada tick, en caso de haber dato disponible, se lo obtiene
            if not image_queue.empty():
                image_data = image_queue.get()
                
            if not lidar_queue.empty():
                lidar_data = lidar_queue.get()
            
            #cuando se tengan ambos datos (imagen y nube de puntos) de un mismo frame, se almacenan ambos
            if ticks > 1 and (image_data.frame == lidar_data.frame):

                #guardar imagen
                image_path =  './%s/%.6d.png' % (images_folder, image_data.frame)
                image_data.save_to_disk(image_path)
                #print('imagen %.6d.bin guardada' % image_data.frame)

                #guardar nube de puntos
                pc = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4')))
                pc = np.reshape(pc, (int(pc.shape[0] / 4), 4))
                pointcloud_path = './%s/%.6d.bin' % (pointclouds_folder, lidar_data.frame) 
                pc.tofile(pointcloud_path)
                #print('point cloud %.6d.bin guardada' % lidar_data.frame)
                 
                frames_captured = frames_captured + 1
        
            sys.stdout.write("\r Capturados %d frames de %d en %d ticks" % (frames_captured,frames,ticks) + ' ')
            sys.stdout.flush()
        
    finally:
        world.apply_settings(original_settings)
        vehicle.destroy()
        lidar.destroy()
        camera.destroy()

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
