import glob
import os
import sys
import argparse
import time
from datetime import datetime
import random
import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt
import open3d as o3d
from kitti_label import KittiLabel, generate_kittilabel
try:
    sys.path.append('../PythonAPI/carla/dist/carla-0.9.13-py3.8-linux-x86_64.egg')
except IndexError:
    pass

import carla

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

args = 0
bounding_boxes = 0

def lidar_callback(point_cloud):
    """Recibe la nube de puntos desde el simulador y la guarda en formato binario"""

    #logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    
    #Crear directorios para guardar los datos, nubes de puntos, imagenes y labels
    images_path,pointclouds_path,calib_path,labels_path = create_output_folders()
    
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    #print(point_cloud)
    total_points = 0
    for i in range(point_cloud.channels):
        total_points += point_cloud.get_point_count(i)
        #print(point_cloud.get_point_count(i))

    points = data[0:total_points*4]
    time_resolved_signals = data[total_points*4::]
    #print(len(points))
    #print(points)
    #print(len(time_resolved_signals))
    #print(time_resolved_signals)
    #for i in data[]:
    #    print(i)
    data = np.reshape(points, (int(points.shape[0] / 4), 4))

    len_signal = int(len(time_resolved_signals)/(total_points))
    #print(len_signal)
    data2 = np.reshape(time_resolved_signals, (int(time_resolved_signals.shape[0] /  len_signal ), len_signal))

    #print(data)
    #print(data2)

    np.savetxt('./logs/time_signal.txt', data2, delimiter=" ", fmt="%s")
    
    data.tofile('./point_clouds/tests/%.6d_%d_%d_%d.bin' % (point_cloud.frame,args.x,args.y,args.z))
    print('point cloud %.6d_%d_%d_%d.bin guardada' % (point_cloud.frame,args.x,args.y,args.z))

    # Load binary point cloud
    bin_pcd = np.fromfile('./point_clouds/tests/%.6d_%d_%d_%d.bin' % (point_cloud.frame,args.x,args.y,args.z), dtype=np.float32)
    
    # Reshape and drop reflection values
    points = bin_pcd.reshape((-1, 4))[:, 0:3]
    
    # Convert to Open3D point cloud
    o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))

    # Save to whatever format you like
    o3d.io.write_point_cloud('./point_clouds/%.6d.pcd' % point_cloud.frame, o3d_pcd)
    print('point cloud %.6d.pcd guardada' % point_cloud.frame)

    print(bounding_boxes)
    
def main(arg):
    """Spawnea el LIDAR HDL-64E y genera un paso de simulacion"""
    #Cliente y simulador
    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)

    world = client.get_world()

    args = arg

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
        lidar_bp.set_attribute('upper_fov', str(16.5))
        lidar_bp.set_attribute('lower_fov', str(-16.5))
        lidar_bp.set_attribute('channels', str(16))
        lidar_bp.set_attribute('range', str(20))
        lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
        lidar_bp.set_attribute('points_per_second', str(1152000))
        lidar_bp.set_attribute('noise_stddev', str(0.01))
        lidar_bp.set_attribute('dropoff_general_rate',str(0.0))
        lidar_bp.set_attribute('tx_fs',str(1e9))
        lidar_bp.set_attribute('ch_fs',str(1e9))
        lidar_bp.set_attribute('rx_fs',str(1e9))
        lidar_bp.set_attribute('debug_global',"true")
        #lidar_bp.set_attribute('debug_rx',"true")
        lidar_bp.set_attribute('log_rx',"false")
        lidar_bp.set_attribute('model_transceptor',"false")
        lidar_bp.set_attribute('model_intensity',"true")
        lidar_bp.set_attribute('model_multiple_return',"false")
        lidar_bp.set_attribute('num_max_returns',"3")
        #lidar_bp.set_attribute('power_tx',str(50e-3))
        lidar_bp.set_attribute('rpd_rx',str(0.1))

        #Spawnea el LIDAR en la simulacion en la ubicacion especificado por argumentos
        lidar_transform = carla.Transform(carla.Location(arg.x, arg.y, arg.z), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))
        lidar = world.spawn_actor(lidar_bp, lidar_transform)

        box_bp = blueprint_library.find('static.prop.box01')
        box_transform = carla.Transform(carla.Location(1.3, -1.3, 0.5), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))
        box = world.spawn_actor(box_bp,box_transform)
        
        global bounding_boxes
        bounding_boxes = box.bounding_box

        #Se crea el label de este npc y se obtiene la informacion necesaria
        #label = generate_kittilabel('Pedestrian',npc, vehicle, world_2_camera,K,image_w,image_h,world_2_lidar,rot_trans_matrix)
        #str_label = label.get_label()
        #label_file.write("{}\n".format(str_label))
        
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
        default=1,
        type=float,
        help='posicion del sensor en el eje Z en metros, default:1.5')
    args = argparser.parse_args()

    try:
        main(args)
    except KeyboardInterrupt:
        print(' - Exited by user.')
