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
    #UNREAL UTILIZA EL SISTEMA X: Foward, Y:Right, Z: Up
    #PERO LA CAMARA EN KITTI UTILIZA X: Right, Y:Down, Z: Forward
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

def w3D_to_lidar3D(loc,w2l):
    #Tranformar coordenada world-3D a lidar-3D

    # Formatear la coordenada de entrada, agrega 1 al final para poder hacer el prod punto
    # (loc is a carla.Position object)
    point = np.array([loc.x, loc.y, loc.z, 1])

    # Trasformar coordenadas de world-3D->lidar-3D (traslacion)
    point_lidar = np.dot(w2l, point)

    # Modificar el sistema de coordenadas de UE4 al de lidar de KITTI
    ##UNREAL UTILIZA EL SISTEMA X: Foward, Y:Right, Z: Up
    #PERO EL LIDAR EN KITTI UTILIZA X: Foward, Y:Left, Z: Up
    # (x, y ,z) -> (x, -y, z)
    # y se elimina el cuarto componente agregado
    point_lidar = [point_lidar[0], -point_lidar[1], point_lidar[2]]

    return point_lidar

def bbox_in_image(bbox_2D,image_w,image_h):

    #Corregir la bbox2d para que este completamente dentro de la imagen
    #bbox_2D = [left_limit, high_limit, rigth_limit, low_limit]
    
    new_left_limit = max(0.0, min(bbox_2D[0], image_w-1.0))
    new_high_limit = max(0.0, min(bbox_2D[1], image_h-1.0))
    new_right_limit = max(0.0, min(bbox_2D[2], image_w-1.0))
    new_low_limit = max(0.0, min(bbox_2D[3], image_h-1.0))

    bbox_2D_in_image = [new_left_limit,new_high_limit,new_right_limit,new_low_limit]

    return bbox_2D_in_image

def is_in_image(npc,K,w2c,image_w,image_h):
    """ Determinar si el vehiculo npc se enceuntra dentro de los limites de la imagen """

    npc_center_image_pos = w3D_to_cam2D(npc.get_transform().location,K,w2c)

    return (npc_center_image_pos[0] > 0.0 and  npc_center_image_pos[0] < image_w and \
            npc_center_image_pos[1] > 0.0 and  npc_center_image_pos[1] < image_h  )

def is_in_front(vehicle,npc):
    """ Determinar si el actor se encuentra al frente del vehiculo q posee la camara """

    #Prod punto entre el forward vector del vehiculo y el vector entre el vehiculo y el actor
    forward_vec = vehicle.get_transform().get_forward_vector()
    ray = npc.get_transform().location - vehicle.get_transform().location

    #si el prod punto es mayor a 1, significa q el actor esta al frente del vehiculo
    return (forward_vec.dot(ray) > 1)

def save_calibration_matrices(filename, intrinsic_mat, extrinsic_mat):
    """ Saves the calibration matrices to a file.
        AVOD (and KITTI) refers to P as P=K*[R;t], so we will just store P.
        The resulting file will contain:
        3x4    p0-p3      Camera P matrix. Contains extrinsic
                          and intrinsic parameters. (P=K*[R;t])
        3x3    r0_rect    Rectification matrix, required to transform points
                          from velodyne to camera coordinate frame.
        3x4    tr_velodyne_to_cam    Used to transform from velodyne to cam
                                     coordinate frame according to:
                                     Point_Camera = P_cam * R0_rect *
                                                    Tr_velo_to_cam *
                                                    Point_Velodyne.
        3x4    tr_imu_to_velo        Used to transform from imu to velodyne coordinate frame. This is not needed since we do not export
                                     imu data.
    """
    # KITTI format -> matrices en una linea y con row-major order

    P = intrinsic_mat
    P = np.column_stack((P, np.array([0, 0, 0])))

    R0 = np.identity(3)

    TR_velodyne = extrinsic_mat

    TR_imu_to_velo = np.identity(3)
    TR_imu_to_velo = np.column_stack((TR_imu_to_velo, np.array([0, 0, 0])))

    def write_matrix(f, name, arr):
        arr_1d = arr.ravel()
        f.write("{}: {}\n".format(name, ' '.join('%1.12e'%num for num in arr_1d)))

    with open(filename, 'w') as f:
        for i in range(4):  # Se llenan las 4 matrices de cada camara, aunque solo se haya utilizado la P2
            write_matrix(f, "P" + str(i), P)
        write_matrix(f, "R0_rect", R0)
        write_matrix(f, "Tr_velo_to_cam", TR_velodyne)
        write_matrix(f, "TR_imu_to_velo", TR_imu_to_velo)