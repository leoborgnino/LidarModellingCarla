import numpy as np
import os
import open3d as o3d
import math
import matplotlib.pyplot as plt
from matplotlib import cm

import kitti_config as cnf

def get_labels(label_filename):
    labels=[]
    with open(label_filename) as f_label:
        lines = f_label.readlines()
        for line in lines:
            line = line.strip('\n').split()
            labels.append(line)
    
    return labels

def get_label_type(label_line):
    return label_line[0]

def get_label_truncated(label_line):
    return float(label_line[1])

def get_label_occluded(label_line):
    return int(label_line[2])

def get_label_alpha(label_line):
    return float(label_line[3])

def get_label_bbox(label_line):
    left=float(label_line[4])
    top=float(label_line[5])
    right=float(label_line[6])
    bottom=float(label_line[7])
    return [left,top,right,bottom]

def get_label_dimensions(label_line):
    height=float(label_line[8])
    width=float(label_line[9])
    length=float(label_line[10])

    return [height,width,length]

def get_label_location(label_line):
    x=float(label_line[11])
    y=float(label_line[12])
    z=float(label_line[13])

    return [x,y,z]

def get_label_rotation(label_line):
    return float(label_line[14])

def get_colors_pointcloud(intensity):
    cmap = cm.get_cmap('plasma')
    VIRIDIS = np.array(cmap(np.arange(0,cmap.N)))

    VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
    #int_color = np.c_[
    #    np.interp(intensity, VID_RANGE, VIRIDIS[:, 2]),
    #    np.interp(intensity, VID_RANGE, VIRIDIS[:, 1]),
    #    np.interp(intensity, VID_RANGE, VIRIDIS[:, 0])]
    
    int_color = np.c_[
        np.interp(intensity, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity, VID_RANGE, VIRIDIS[:, 2])]
    
    return int_color


#DATA_DIR = '/home/gaston/Documents/Kitti' 
DATA_DIR = '/media/gaston/HDD-Ubuntu/carla/ScriptsPruebas/scripts/28-03-23_16:57:59/' 
#DATA_DIR = '/media/gaston/HDD-Ubuntu/carla/ScriptsPruebas/Complex-YOLOv4-Pytorch/dataset/kitti/'
IMG_DIR = DATA_DIR + '/image_2'
#LABEL_DIR = DATA_DIR + '/training_filter_minpoints_C2P2C2/label_2'
LABEL_DIR = DATA_DIR + '/training/label_2'
POINT_CLOUD_DIR = DATA_DIR + '/training/velodyne'
#POINT_CLOUD_DIR = DATA_DIR + '/velodyne_carla'
CALIB_DIR = DATA_DIR + '/calib'

def main():

    print('Labels directory: ' + LABEL_DIR)
    list_files=os.listdir(LABEL_DIR)
    
    list_files=[x.split('.')[0] for x in list_files]
    list_files.sort()
    #list_files=[1796]

    cant_points_in_bbox = []
    cant_points_in_pc = []
    #Para cada dato
    for file in list_files:

        file_id = int(file)

        img_filename = os.path.join(IMG_DIR, '{0:06d}.png'.format(file_id))
        label_filename = os.path.join(LABEL_DIR, '{0:06d}.txt'.format(file_id))
        pc_filename = os.path.join(POINT_CLOUD_DIR, '{0:06d}.bin'.format(file_id))
        calib_filename = os.path.join(CALIB_DIR, '{0:06d}.txt'.format(file_id))

        #Se obtienen todos los labels (cada linea del label file)
        labels = get_labels(label_filename)

        #Se carga la nube de puntos
        data = np.reshape(np.fromfile(pc_filename, '<f4'), (-1, 4))
        points = data[:, :-1]
        intensity = data[:, -1]

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)
        pc.colors = o3d.utility.Vector3dVector(get_colors_pointcloud(intensity))

        cant_points = len(pc.points)
        cant_points_in_pc.append(cant_points)

        origin = o3d.geometry.TriangleMesh.create_coordinate_frame()

        orientation_test = 0.0
        R_test = o3d.geometry.get_rotation_matrix_from_xyz(np.asarray([0,0, orientation_test]))
        extent_test = np.array([4.7,2.3,1.73]) #dimesiones en x,y,z (coordenadas de lidar)
        center_test = np.array([0.0,0.0,-1.73/2]) #ubicacion del centro en x,y,z (coordenadas de lidar)
        obb_test = o3d.geometry.OrientedBoundingBox(center_test,R_test,extent_test)
        
        #Se analiza cada label
        geometries = []
        geometries.append(pc)
        geometries.append(origin)
        geometries.append(obb_test)

        for label in labels:
            type = get_label_type(label)
            if(type == 'Car' or type == 'Pedestrian' or type == 'Cyclist'):
                location = get_label_location(label)
                rotation = get_label_rotation(label)
                dimensions = get_label_dimensions(label)

                location.append(1.)

                #Transformacion de cam to lidar
                location_lidar = np.matmul(cnf.R0_inv, location)
                location_lidar = np.matmul(cnf.Tr_velo_to_cam_inv, location_lidar)
                location_lidar = location_lidar[0:3]

                orientation = -(rotation + np.pi/2)
                location_lidar[2] = location_lidar[2] + dimensions[0]/2
                R = o3d.geometry.get_rotation_matrix_from_xyz(np.asarray([0,0, orientation]))
                extent = np.array([dimensions[2],dimensions[1],dimensions[0]]) #dimesiones en x,y,z (coordenadas de lidar)
                center = np.array(location_lidar) #ubicacion del centro en x,y,z (coordenadas de lidar)
                obb = o3d.geometry.OrientedBoundingBox(center,R,extent)

                #inside_indices = obb.get_point_indices_within_bounding_box(pc.points)
                #cant_points_inside = len(inside_indices)
                #cant_points_in_bbox.append(cant_points_inside)
                geometries.append(obb)

        
        o3d.visualization.draw_geometries(geometries)

if __name__ == "__main__":
    try: 
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')