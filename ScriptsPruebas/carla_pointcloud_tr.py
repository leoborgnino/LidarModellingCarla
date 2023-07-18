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

try:
    sys.path.append('../PythonAPI/carla/dist/carla-0.9.13-py3.8-linux-x86_64.egg')
except IndexError:
    
    pass

import carla

def lidar_callback(point_cloud):
    """Recibe la nube de puntos desde el simulador y la guarda en formato binario"""

    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    data = np.reshape(data, (int(data.shape[0] / 4), 4))

    data.tofile('./point_clouds/%.6d.bin' % point_cloud.frame)
    print('point cloud %.6d.bin guardada' % point_cloud.frame)

    # Load binary point cloud
    bin_pcd = np.fromfile('./point_clouds/%.6d.bin' % point_cloud.frame, dtype=np.float32)
    
    # Reshape and drop reflection values
    points = bin_pcd.reshape((-1, 4))[:, 0:3]
    
    # Convert to Open3D point cloud
    o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))

    # Save to whatever format you like
    o3d.io.write_point_cloud('./point_clouds/%.6d.pcd' % point_cloud.frame, o3d_pcd)
    print('point cloud %.6d.pcd guardada' % point_cloud.frame)
    
def main(arg):
    """Spawnea el LIDAR HDL-64E y genera un paso de simulacion"""
    #Cliente y simulador
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()

    try:
        #Setea modo sincrono en la simulacion con delta fijo
        original_settings = world.get_settings()
        settings = world.get_settings()
        delta = 0.05
        settings.fixed_delta_seconds = delta
        settings.synchronous_mode = True
        world.apply_settings(settings)

        #Crea el LIDAR con las especificaciones de HDL-64E
        blueprint_library = world.get_blueprint_library()
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast_time_resolved')
        lidar_bp.set_attribute('upper_fov', str(2.0))
        lidar_bp.set_attribute('lower_fov', str(-24.8))
        lidar_bp.set_attribute('channels', str(16))
        lidar_bp.set_attribute('range', str(40))
        lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
        lidar_bp.set_attribute('points_per_second', str(100000))
        lidar_bp.set_attribute('noise_stddev', str(0.01))
        lidar_bp.set_attribute('dropoff_general_rate',str(0.0))
        lidar_bp.set_attribute('debug_global',"false")
        lidar_bp.set_attribute('power_tx',str(50e-3))

        #Spawnea el LIDAR en la simulacion en la ubicacion especificado por argumentos
        lidar_transform = carla.Transform(carla.Location(arg.x, arg.y, arg.z), 
                            carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))
        lidar = world.spawn_actor(lidar_bp, lidar_transform)

        #Define la funcion de callback al recibir una nube de puntos
        lidar.listen(lambda data: lidar_callback(data))
        
        world.tick()
        time.sleep(10.0)

    finally:
        world.apply_settings(original_settings)
        lidar.destroy()

if __name__ == "__main__":
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '-x',
        default=0.0,
        type=float,
        help='posicion del sensor en el eje X en metros, default:0.0')
    argparser.add_argument(
        '-y',
        default=0.0,
        type=float,
        help='posicion del sensor en el eje Y en metros, default:0.0')
    argparser.add_argument(
        '-z',
        default=1.5,
        type=float,
        help='posicion del sensor en el eje Z en metros, default:1.5')
    args = argparser.parse_args()

    try:
        main(args)
    except KeyboardInterrupt:
        print(' - Exited by user.')
