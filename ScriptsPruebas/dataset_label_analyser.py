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

def plot_histogram(array_data, title):
    intervalos = range(min(array_data), max(array_data)+2)

    plt.hist(array_data, bins=intervalos, alpha=0.5)
    plt.title(title)
    plt.xlabel('variable X')
    plt.ylabel('count')
    plt.xticks(intervalos)

    plt.show()


#DATA_DIR = '/home/gaston/Documents/Kitti/training' 
#DATA_DIR = '/home/gaston/Documents/Kitti/training_filter_minpoints_C2P2C2' 
DATA_DIR = '/media/gaston/HDD-Ubuntu/carla/ScriptsPruebas/Dataset_sintetico/training'
#DATA_DIR = '/media/gaston/HDD-Ubuntu/carla/ScriptsPruebas/Dataset_sintetico/training_filter_minpoints_C3P3C3' 
#DATA_DIR = '/media/gaston/HDD-Ubuntu/carla/ScriptsPruebas/28-02-23_13:43:31/training_filter_minpoints_C2P2C2' 

 
IMG_DIR = DATA_DIR + '/image_2'
LABEL_DIR = DATA_DIR + '/label_2'
POINT_CLOUD_DIR = DATA_DIR + '/velodyne'
CALIB_DIR = DATA_DIR + '/calib'

def main():

    print('Labels directory: ' + LABEL_DIR)
    list_files=os.listdir(LABEL_DIR)
    
    list_files=[x.split('.')[0] for x in list_files]
    #list_files = [96]
    cant_data = 0

    cant_car_total = 0
    cant_pedestrian_total = 0
    cant_cyclist_total = 0

    cant_car_per_data = []
    cant_pedestrian_per_data = []
    cant_cyclist_per_data = []

    #Para cada dato
    for file in list_files:

        file_id = int(file)
        
        img_filename = os.path.join(IMG_DIR, '{0:06d}.png'.format(file_id))
        label_filename = os.path.join(LABEL_DIR, '{0:06d}.txt'.format(file_id))
        pc_filename = os.path.join(POINT_CLOUD_DIR, '{0:06d}.bin'.format(file_id))
        calib_filename = os.path.join(CALIB_DIR, '{0:06d}.txt'.format(file_id))

        #Se obtienen todos los labels (cada linea del label file)
        labels = get_labels(label_filename)
        cant_data +=1

        cant_car= 0
        cant_pedestrian= 0
        cant_cyclist= 0

        #Se analiza cada label
        for label in labels:
            type = get_label_type(label)
            if(type == 'Car' ):
                cant_car +=1 
            if(type == 'Pedestrian'):
                cant_pedestrian+=1
            if(type == 'Cyclist'):
                cant_cyclist+=1
        
        cant_car_total+=cant_car
        cant_pedestrian_total+=cant_pedestrian
        cant_cyclist_total+=cant_cyclist

        cant_car_per_data.append(cant_car)
        cant_pedestrian_per_data.append(cant_pedestrian)
        cant_cyclist_per_data.append(cant_cyclist)
  
    print('Se analizaron: {} labels'.format(cant_data))
    print('Cantidad de Car: {}'.format(cant_car_total))
    print('Cantidad de Pedestrian: {}'.format(cant_pedestrian_total))
    print('Cantidad de Cyclist: {}'.format(cant_cyclist_total))

    print('En un solo frame:')
    print('Maxima cantidad de car: {}'.format(max(cant_car_per_data)))
    print('Min cantidad de car: {}'.format(min(cant_car_per_data)))
    print('Maxima cantidad de pedestrian: {}'.format(max(cant_pedestrian_per_data)))
    print('Min cantidad de pedestrian: {}'.format(min(cant_pedestrian_per_data)))
    print('Maxima cantidad de cyclist: {}'.format(max(cant_cyclist_per_data)))
    print('Min cantidad de cyclist: {}'.format(min(cant_cyclist_per_data)))

    plot_histogram(cant_car_per_data,'Carla - cantidad de Car por frame en {} frames'.format(cant_data))
    plot_histogram(cant_pedestrian_per_data,'Carla - cantidad de Pedestrian por frame en {} frames'.format(cant_data))
    plot_histogram(cant_cyclist_per_data,'Carla - cantidad de Cyclist por frame en {} frames'.format(cant_data))




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
    print('Cantidad de bbox Pedestrian analizadas: ' + str(len(cant_points_in_bbox_pedestrian)))
    plot_histogram(cant_points_in_bbox_pedestrian, 'Cantidad de puntos en el bbox Pedestrian')
    '''



if __name__ == "__main__":
    try: 
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')