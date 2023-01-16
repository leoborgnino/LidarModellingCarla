import numpy as np
from numpy.linalg import pinv, inv

def build_projection_matrix(w, h, fov):
    """ Calcula la matriz de proyeccion K segun el fov, height y width de la imagen """
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K

def w3D_to_cam3D(loc,w2c):
    #Tranformar coordenada world-3D a camara-3D

    # Formatear la coordenada de entrada, agrega 1 al final para poder hacer el prod punto
    # (loc is a carla.Position object)
    point = np.array([loc.x, loc.y, loc.z, 1])

    # Trasformar coordenadas de world-3D->camara-3D (traslacion)
    point_camera = np.dot(w2c, point)

    # Modificar el sistema de coordenadas de UE4 al standard de camara (rotacion)
    # (x, y ,z) -> (y, -z, x)
    # y se elimina el cuarto componente agregado
    point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

    return point_camera

def w3D_to_cam2D(loc, K, w2c):

        #Tranformar coordenada world-3D a camara-2D

        point_camera = w3D_to_cam3D(loc,w2c)

        # Proyeccion 3D->2D utilizando la matriz K
        point_img = np.dot(K, point_camera)
        # Normalizar
        point_img[0] /= point_img[2]
        point_img[1] /= point_img[2]

        return point_img[0:2]

def bbox_in_image(bbox_2D,image_w,image_h):

    #Corregir la bbox2d para que este completamente dentro de la imagen
    #bbox_2D = [left_limit, high_limit, rigth_limit, low_limit]
    
    new_left_limit = max(0.0, min(bbox_2D[0], image_w-1.0))
    new_high_limit = max(0.0, min(bbox_2D[1], image_h-1.0))
    new_right_limit = max(0.0, min(bbox_2D[2], image_w-1.0))
    new_low_limit = max(0.0, min(bbox_2D[3], image_h-1.0))

    bbox_2D_in_image = [new_left_limit,new_high_limit,new_right_limit,new_low_limit]

    return bbox_2D_in_image

