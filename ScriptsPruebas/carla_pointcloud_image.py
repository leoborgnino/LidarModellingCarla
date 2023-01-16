import glob
import os
import sys
import argparse
import time
from datetime import datetime
import random
import numpy as np
from matplotlib import cm
import open3d as o3d
from queue import Queue
from queue import Empty

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
    client.set_timeout(2.0)
    world = client.get_world()

    try:
        #Setea modo sincrono en la simulacion con delta fijo
        original_settings = world.get_settings()
        settings = world.get_settings()
        delta = 0.05
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
        lidar_bp.set_attribute('noise_stddev', str(0.01))
        lidar_bp.set_attribute('dropoff_general_rate',str(0.0))

        #Crea la camara RGB
        camera_bp = blueprint_library.filter("sensor.camera.rgb")[0]
        camera_bp.set_attribute("image_size_x", str(1242)) #Resolucion de las imagenes de Kitti
        camera_bp.set_attribute("image_size_y", str(375))

        #Crea vehiculo sobre el cual montar los sensores
        vehicle_bp = blueprint_library.filter("vehicle.lincoln.mkz_2017")[0]

        #Spawnear vehiculo y sensores
        vehicle_transform = world.get_map().get_spawn_points()[0]
        vehicle = world.spawn_actor(
            blueprint=vehicle_bp,
            transform=vehicle_transform)
        vehicle.set_autopilot(True)

        camera = world.spawn_actor(
            blueprint=camera_bp,
            transform=carla.Transform(carla.Location(x=0.5, z=1.65)), #posicion segun Kitti
            attach_to=vehicle)

        lidar = world.spawn_actor(
            blueprint=lidar_bp,
            transform=carla.Transform(carla.Location(x=1.0, z=1.73)), #posicion segun Kitti
            attach_to=vehicle)

        #Funciones de callback para almacenar imagen y nube de puntos
        image_queue= Queue()
        lidar_queue= Queue()

        lidar.listen(lambda data: sensor_callback(data,lidar_queue))
        camera.listen(lambda data: sensor_callback(data,image_queue))
        
        world.tick()
        
        image_data = image_queue.get()
        lidar_data = lidar_queue.get()
        
        #guardar nube de puntos
        pc = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4')))
        pc = np.reshape(pc, (int(pc.shape[0] / 4), 4))

        pc.tofile('./point_clouds/%.6d.bin' % lidar_data.frame)
        print('point cloud %.6d.bin guardada' % lidar_data.frame)

        #guardar imagen
        image_data.save_to_disk('./images/%6d.png' % image_data.frame)
        print('imagen %.6d.bin guardada' % image_data.frame)

    finally:
        world.apply_settings(original_settings)
        vehicle.destroy()
        lidar.destroy()
        camera.destroy()

if __name__ == "__main__":
    argparser = argparse.ArgumentParser(
        description=__doc__)
    args = argparser.parse_args()

    try:
        main(args)
    except KeyboardInterrupt:
        print(' - Exited by user.')
