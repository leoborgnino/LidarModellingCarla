import numpy as np
import os
import open3d as o3d
import math
import matplotlib.pyplot as plt

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

def plot_histogram(array_data, title,cant_bins=100):
    #Plotear histograma
    bins = np.linspace(math.ceil(min(array_data)), 
                   math.floor(max(array_data)),
                   cant_bins) # fixed number of bins
    plt.xlim([min(array_data)-1, max(array_data)+1])
    plt.hist(array_data, bins=bins, alpha=0.5)
    plt.title(title)
    plt.xlabel('variable X ({} evenly spaced bins)'.format(cant_bins))
    plt.ylabel('count')

    plt.show()


DATA_DIR = '/home/gaston/Documents/Kitti/training' 
#DATA_DIR = '/media/gaston/HDD-Ubuntu/carla/ScriptsPruebas/21-02-23_19:47:04/training' 
IMG_DIR = DATA_DIR + '/image_2'
LABEL_DIR = DATA_DIR + '/label_2'
POINT_CLOUD_DIR = DATA_DIR + '/velodyne'
CALIB_DIR = DATA_DIR + '/calib'

def main():

    print('Labels directory: ' + LABEL_DIR)
    list_files=os.listdir(LABEL_DIR)
    
    list_files=[x.split('.')[0] for x in list_files]
    #list_files = [96]

    cant_points_in_bbox_car = []
    cant_points_in_bbox_pedestrian = []
    cant_points_in_bbox_ciclyst = []
    cant_points_in_pc = []
    distancias_pedestrian = []
    intensidad_prom_pedestrian = []
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

        cant_points = len(pc.points)
        if(cant_points > 70000):
            cant_points_in_pc.append(cant_points)

        print(cant_points)

        #origin = o3d.geometry.TriangleMesh.create_coordinate_frame()
        #o3d.visualization.draw_geometries([pc,origin])

        #Se analiza cada label
        for label in labels:
            type = get_label_type(label)
            if(type == 'Car' or type == 'Pedestrian'):
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

                inside_indices = obb.get_point_indices_within_bounding_box(pc.points)
                cant_points_inside = len(inside_indices)
                if(type=='Car'):
                    if(cant_points_inside < 10):
                        print(file_id)
                        #o3d.visualization.draw_geometries([pc,obb])
                    cant_points_in_bbox_car.append(cant_points_inside)
                if(type=='Pedestrian'):
                    print('pedestrian detectado')
                    if(cant_points_inside < 100):
                    
                        cant_points_in_bbox_pedestrian.append(cant_points_inside)
                    if(cant_points_inside < 20):
                        print(file_id)
                        #o3d.visualization.draw_geometries([pc,obb])
                    distancia = math.sqrt(location_lidar[0]**2 + location_lidar[1]**2 + location_lidar[2]**2)
                    if(cant_points_inside > 0):
                        prom=0
                        for inside_indice in inside_indices:
                            prom += intensity[inside_indice]
                        prom /= cant_points_inside
                        distancias_pedestrian.append(distancia)
                        intensidad_prom_pedestrian.append(prom)
                if(type=='Cyclist'):
                    cant_points_in_bbox_ciclyst.append(cant_points_inside)
    '''
    print('pedestrians analizados: {}'.format(len(intensidad_prom_pedestrian)))
    print(distancias_pedestrian)
    print(intensidad_prom_pedestrian)

    fig, ax = plt.subplots()
    ax.scatter(distancias_pedestrian,intensidad_prom_pedestrian)
    plt.show()

       
    print('Cantidad de pointclouds analizadas: ' + str(len(cant_points_in_pc)))
    plot_histogram(cant_points_in_pc, 'Cantidad de puntos en pointclouds')
    print('Cantidad de bbox Car analizadas: ' + str(len(cant_points_in_bbox_car)))
    plot_histogram(cant_points_in_bbox_car, 'Cantidad de puntos en el bbox Car')
    '''
    print('Cantidad de bbox Pedestrian analizadas: ' + str(len(cant_points_in_bbox_pedestrian)))
    plot_histogram(cant_points_in_bbox_pedestrian, 'Cantidad de puntos en el bbox Pedestrian',100)
    print(min(cant_points_in_bbox_pedestrian))
    print(min(cant_points_in_bbox_ciclyst))




if __name__ == "__main__":
    try: 
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')