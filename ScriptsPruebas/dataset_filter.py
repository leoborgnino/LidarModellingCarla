import numpy as np
import os
import open3d as o3d
import math
import matplotlib.pyplot as plt

from kitti_label import KittiLabel, generate_kittilabel_from_line

def inverse_rigid_trans(Tr):
    ''' Inverse a rigid body transform matrix (3x4 as [R|t])
        [R'|-R't; 0|1]
    '''
    inv_Tr = np.zeros_like(Tr)  # 3x4
    inv_Tr[0:3, 0:3] = np.transpose(Tr[0:3, 0:3])
    inv_Tr[0:3, 3] = np.dot(-np.transpose(Tr[0:3, 0:3]), Tr[0:3, 3])
    return inv_Tr

def read_calib_file(filepath):
    with open(filepath) as f:
        lines = f.readlines()

    obj = lines[2].strip().split(' ')[1:]
    P2 = np.array(obj, dtype=np.float32)
    obj = lines[3].strip().split(' ')[1:]
    P3 = np.array(obj, dtype=np.float32)
    obj = lines[4].strip().split(' ')[1:]
    R0 = np.array(obj, dtype=np.float32)
    obj = lines[5].strip().split(' ')[1:]
    Tr_velo_to_cam = np.array(obj, dtype=np.float32)

    calibs = {'P2': P2.reshape(3, 4),
            'P3': P3.reshape(3, 4),
            'R_rect': R0.reshape(3, 3),
            'Tr_velo2cam': Tr_velo_to_cam.reshape(3, 4)}

    P = calibs['P2']
    P = np.reshape(P, [3, 4])
    V2C = calibs['Tr_velo2cam']
    V2C = np.reshape(V2C, [3, 4])
    C2V = inverse_rigid_trans(V2C)
    R0 = calibs['R_rect']
    R0 = np.reshape(R0, [3, 3])
    return P,V2C,C2V,R0

def get_labels(label_filename):
    labels=[]
    with open(label_filename) as f_label:
        lines = f_label.readlines()
        for line in lines:
            line = line.strip('\n').split()
            labels.append(line)
    
    return labels

def create_folder(output_directory,folder):
    output_folder = os.path.join(output_directory,folder)
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    return output_folder

def get_cant_points_in_bbox(pc,kitti_label,V2C,C2V,R0):
    location = kitti_label.location
    rotation = kitti_label.rotation_y
    dimensions = kitti_label.dimensions

    location.append(1.)

    #Transformacion de cam to lidar
    R0_i = np.zeros((4, 4))
    R0_i[:3, :3] = R0
    R0_i[3, 3] = 1
    location_lidar = np.matmul(np.linalg.inv(R0_i), location)
    location_lidar = np.matmul(C2V, location_lidar)
    location_lidar = location_lidar[0:3]

    orientation = -(rotation + np.pi/2)
    location_lidar[2] = location_lidar[2] + dimensions[0]/2
    R = o3d.geometry.get_rotation_matrix_from_xyz(np.asarray([0,0, orientation]))
    extent = np.array([dimensions[2],dimensions[1],dimensions[0]]) #dimesiones en x,y,z (coordenadas de lidar)
    center = np.array(location_lidar) #ubicacion del centro en x,y,z (coordenadas de lidar)
    obb = o3d.geometry.OrientedBoundingBox(center,R,extent)

    inside_indices = obb.get_point_indices_within_bounding_box(pc.points)
    #print(len(inside_indices))
    o3d.visualization.draw_geometries([pc,obb])

    return len(inside_indices)

DATA_DIR = '/home/gaston/Documents/Kitti/training'
#DATA_DIR = '/media/gaston/HDD-Ubuntu/carla/ScriptsPruebas/Dataset_sintetico/training' 
IMG_DIR = DATA_DIR + '/image_2'
LABEL_DIR = DATA_DIR + '/label_2'
POINT_CLOUD_DIR = DATA_DIR + '/velodyne'
CALIB_DIR = DATA_DIR + '/calib'

def main():

    print('Labels directory: ' + LABEL_DIR)
    list_files=os.listdir(LABEL_DIR)
    list_files=[x.split('.')[0] for x in list_files]
    list_files.sort()

    #CANTIDADES MINIMAS DE PUNTOS
    min_points_Car = 3
    min_points_Pedestrian = 3
    min_points_Cyclist = 3


    #Path del nue
    new_data_dir = DATA_DIR + '_filter_minpoints_C{}P{}C{}'.format(min_points_Car,min_points_Pedestrian,min_points_Cyclist)
    new_label_dir = create_folder(new_data_dir,'label_2')
    
    print(new_label_dir)

    #Para cada dato
    for file in list_files:

        file_id = int(file)
        #print(file_id)

        img_filename = os.path.join(IMG_DIR, '{0:06d}.png'.format(file_id))
        label_filename = os.path.join(LABEL_DIR, '{0:06d}.txt'.format(file_id))
        pc_filename = os.path.join(POINT_CLOUD_DIR, '{0:06d}.bin'.format(file_id))
        calib_filename = os.path.join(CALIB_DIR, '{0:06d}.txt'.format(file_id))

        #calib file
        P,V2C,C2V,R0=read_calib_file(calib_filename)
        print(read_calib_file(calib_filename))

        new_label_filename = os.path.join(new_label_dir, '{0:06d}.txt'.format(file_id))
        new_label_file = open(new_label_filename,'w')

        #Se obtienen todos los labels (cada linea del label file)
        labels = get_labels(label_filename)

        #Se carga la nube de puntos
        data = np.reshape(np.fromfile(pc_filename, '<f4'), (-1, 4))
        points = data[:, :-1]
        intensity = data[:, -1]

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)

        #origin = o3d.geometry.TriangleMesh.create_coordinate_frame()
        #o3d.visualization.draw_geometries([pc,origin])

        #Se analiza cada label
        for label in labels:
            kitti_label = generate_kittilabel_from_line(label)
            
            #Segun el tipo del label, se filtra segun una cantidad de puntos
            if(kitti_label.type == 'Car'):
                cant_points_inside = get_cant_points_in_bbox(pc,kitti_label,V2C,C2V,R0)
                if(cant_points_inside >= min_points_Car):
                    new_label_file.write("{}\n".format(kitti_label.get_label()))
                #else:
                    #o3d.visualization.draw_geometries([pc,obb])
                    #print("Label filtrado porque no supero el minimo de puntos en label: {0:06d}".format(file_id))
            
            elif(kitti_label.type == 'Pedestrian'):
                cant_points_inside = get_cant_points_in_bbox(pc,kitti_label,V2C,C2V,R0)
                if(cant_points_inside >= min_points_Pedestrian):
                    new_label_file.write("{}\n".format(kitti_label.get_label()))
            
            elif(kitti_label.type == 'Cyclist'):
                cant_points_inside = get_cant_points_in_bbox(pc,kitti_label,V2C,C2V,R0)
                if(cant_points_inside >= min_points_Cyclist):
                    new_label_file.write("{}\n".format(kitti_label.get_label()))

            else:
                new_label_file.write("{}\n".format(kitti_label.get_label()))

        new_label_file.close()


if __name__ == "__main__":
    try: 
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')