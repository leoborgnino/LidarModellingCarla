"""
#Values    Name      Description
----------------------------------------------------------------------------
   1    type         Describes the type of object: 'Car', 'Van', 'Truck',
                     'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
                     'Misc' or 'DontCare'
   1    truncated    Float from 0 (non-truncated) to 1 (truncated), where
                     truncated refers to the object leaving image boundaries
   1    occluded     Integer (0,1,2,3) indicating occlusion state:
                     0 = fully visible, 1 = partly occluded
                     2 = largely occluded, 3 = unknown
   1    alpha        Observation angle of object, ranging [-pi..pi]
   4    bbox         2D bounding box of object in the image (0-based index):
                     contains left, top, right, bottom pixel coordinates
   3    dimensions   3D object dimensions: height, width, length (in meters)
   3    location     3D object location x,y,z in camera coordinates (in meters)
   1    rotation_y   Rotation ry around Y-axis in camera coordinates [-pi..pi]
   1    score        Only for results: Float, indicating confidence in
                     detection, needed for p/r curves, higher is better.
"""

import math
from transformation_utils import w3D_to_cam2D, w3D_to_cam3D, bbox_in_image,w3D_to_lidar3D
import numpy as np

#Clase responsable de almacenar los datos necesarios para genere el label en KITTI format
class KittiLabel:
    def __init__(self, type=None, truncated=0.0, occluded=3, alpha=0.0, bbox=None, dimensions=None, location=None, rotation_y=0.0):
        self.type = type
        self.truncated = truncated
        self.occluded = occluded
        self.alpha = alpha
        self.bbox = bbox
        self.dimensions = dimensions
        self.location = location
        self.rotation_y = rotation_y
        self._valid_classes = ['Car', 'Van', 'Truck',
                               'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
                               'Misc', 'DontCare']

    def set_type(self, obj_type: str):
        #assert obj_type in self._valid_classes, "Campo type no valido, debe ser uno de: {}".format(
        #    self._valid_classes)
        self.type = obj_type

    def set_truncated(self, truncated: float):
        if(self.type != 'DontCare'):
            assert 0 <= truncated <= 1, """Campo truncated debe ser Float entre 0 y 1"""
        self.truncated = truncated

    def set_occluded(self, occluded: int):
        if(self.type != 'DontCare'): 
            assert occluded in range(0, 4), """Campo Occluded debe ser Integer (0,1,2,3)"""
        self._occluded = occluded

    def set_alpha(self, alpha: float):
        if(self.type != 'DontCare'):
            assert -math.pi <= alpha <= math.pi, "Alpha debe estar entre [-pi..pi], alpha={}, rot_y={}, x={}, z={}".format(alpha,self.rotation_y,self.location[0],self.location[2])
        self.alpha = alpha

    def set_bbox(self, bbox):
        assert len(bbox) == 4, """ Campo Bbox debe ser el cuadro delimitador del objeto en la imagen 2D. 
                                Contiene las coordenadas de los píxeles de borde"""
        self.bbox = bbox

    def set_dimensions(self, dimensions):
        assert len(dimensions) == 3, """ Campo Dimensions debe ser alto,ancho,largo del objeto en metros """

        #Kitti expects height, width and length (z, y, x):
        self.dimensions = dimensions


    def set_location(self, obj_location):
        assert len(obj_location) == 3, """ Campo Location debe contener las coordenadas x,y,z del centro del objeto en coordenadas de camara"""
        self.location = obj_location

    def set_rotation_y(self, rotation_y: float):
        if(self.type != 'DontCare'):
            assert -math.pi <= rotation_y <= math.pi, "Rotation y debe estar en rando [-pi..pi], rot={}".format(rotation_y)
        self.rotation_y = rotation_y

    def get_label(self):
        """ generar el string correspondiente al label con toda la informacion necesaria"""
        str_truncated = "{:.2f}".format(self.truncated)
        str_occluded = "{}".format(self.occluded)
        str_alpha = "{:.2f}".format(self.alpha)
        str_bbox = "{0:.2f} {1:.2f} {2:.2f} {3:.2f}".format(self.bbox[0], self.bbox[1], self.bbox[2], self.bbox[3])
        str_dimensions = "{0:.2f} {1:.2f} {2:.2f}".format(self.dimensions[0], self.dimensions[1], self.dimensions[2])
        str_location = "{0:.2f} {1:.2f} {2:.2f}".format(self.location[0], self.location[1], self.location[2])
        str_rotation = "{:.2f}".format(self.rotation_y)

        if(self.type != 'DontCare'):
            return "{} {} {} {} {} {} {} {}".format(self.type, str_truncated, str_occluded, str_alpha, str_bbox, str_dimensions, str_location, str_rotation)
        else:
            return "{} -1 -1 -10 {} -1 -1 -1 -1000 -1000 -1000 -10".format(self.type, str_bbox)
    def __str__(self):
        """ Returns the kitti formatted string of the datapoint if it is valid (all critical variables filled out), else it returns an error."""

        if self.bbox is None:
            bbox_format = " "
        else:
            bbox_format = " ".join([str(x) for x in self.bbox])

        return "{} {} {} {} {} {} {} {}".format(self.type, self.truncated, self.occluded, self.alpha, bbox_format, self.dimensions, self.location, self.rotation_y)

def calculate_location(npc,world_2_camera,world_2_lidar,rot_trans_matrix):
    #Location
    #Se obtiene la posicion del centro del vehiculo en coordenadas world3D
    npc_world_center_pos = npc.get_transform().location
    #--------------------------
    #print(npc.get_transform().location)
    #print(world_2_lidar)
    location_lidar = w3D_to_lidar3D(npc_world_center_pos,world_2_lidar)
    location_lidar.append(1.0)
    location = np.matmul(rot_trans_matrix,location_lidar)
    location = location[0:3]
    location[0] = location[0] - 0.1
    #----------------------------
    #Se tranforma de world-3D a camara-3D
    #location = w3D_to_cam3D(npc_world_center_pos,world_2_camera)

    return location

def calculate_dimensions(npc):
    #Bounding box 3D del vehiculo, se obtienen sus dimensiones
    bb = npc.bounding_box
    #https://carla.readthedocs.io/en/0.9.5/measurements/
    length=bb.extent.x*2    #extent.x es la mitad del largo del bb
    width=bb.extent.y*2     #extent.y es la mitad del ancho del bb
    height=bb.extent.z*2    #extent.z es la mitad del alto del bb

    #Dimensions
    dimensions = [height, width, length]
    return dimensions

def calculate_dimensions_bb(bb):
    #https://carla.readthedocs.io/en/0.9.5/measurements/
    length=bb.extent.x*2    #extent.x es la mitad del largo del bb
    width=bb.extent.y*2     #extent.y es la mitad del ancho del bb
    height=bb.extent.z*2    #extent.z es la mitad del alto del bb

    #Dimensions
    dimensions = [height, width, length]
    return dimensions

def calculate_bbox2D(npc,K,world_2_camera,image_w,image_h):
    #Bounding box 3D del vehiculo
    bb = npc.bounding_box
    #Bbox 2D
    #obtener vertices del bb en 2D
    verts = [v for v in bb.get_world_vertices(npc.get_transform())]
    verts_x = []
    verts_y = []
    for vert in verts:
        vert_2D = w3D_to_cam2D(vert, K, world_2_camera)
        verts_x.append(vert_2D[0])
        verts_y.append(vert_2D[1])

    #Coordenada mas a la izquierda
    left_limit = min(verts_x)
    #Coordenada mas a la derecha
    rigth_limit = max(verts_x)
    #Coordenada mas arriba
    high_limit = min(verts_y)
    #Coordenada mas abajo
    low_limit = max(verts_y)
    #en los min y max, ya se tienen los vertices 2D
    bbox_2D = [left_limit, high_limit, rigth_limit, low_limit]
    #Chequear si los limites estan fuera de la imagen, en tal caso llevarlo al valor minimo o maximo segun corresponda
    safe_bbox_2D = bbox_in_image(bbox_2D,image_w,image_h)

    return safe_bbox_2D,bbox_2D

def calculate_rotationy(npc,vehicle):
    #rotation_y
    #https://blog.csdn.net/qq_16137569/article/details/118873033
    rot_npc= npc.get_transform().rotation.yaw
    rot_vehicle = vehicle.get_transform().rotation.yaw
    rot_y = (rot_vehicle - rot_npc + 90.0) 

    if(rot_y > 180.0):
        rot_y -= 360.0
    if(rot_y < -180.0):
        rot_y += 360.0

    rot_y_rad = math.radians(-rot_y)#de grados a radianes
    return rot_y_rad

def calculate_alpha(rotation_y,location):
    #Alpha 
    #https://arxiv.org/abs/2112.04421
    #Se lo calcula a partir de la rot_y y location
    #location = [x,y,z]
    #alpha = rot_y - arctan(x/z)
    alpha = rotation_y - math.atan(location[0]/location[2])
    
    if(alpha > math.pi):
        alpha -= 2*math.pi
    if(alpha < -math.pi):
        alpha += 2*math.pi

    return alpha

def calculate_truncated(bbox_2D, safe_bbox_2D):
    #Truncated
    #Calcular area de las bbox2D, la original y la dentro de la imagen
    #bbox_2D = [left_limit, high_limit, rigth_limit, low_limit]
    bbox_2D_area = (bbox_2D[2]-bbox_2D[0])*(bbox_2D[3]-bbox_2D[1])
    safe_bbox_2D_area = (safe_bbox_2D[2]-safe_bbox_2D[0])*(safe_bbox_2D[3]-safe_bbox_2D[1])

    truncated = 1.0 - (safe_bbox_2D_area/bbox_2D_area) #si las bbox son iguales, es decir no hay truncado, truncated da 0.0
                                                        #mientras mas chico safe_bbox, es decir mas truncado, truncated aumenta 
    return truncated

def generate_kittilabel(type, npc, vehicle, world_2_camera,K,image_w,image_h,world_2_lidar,rot_trans_matrix):
    label = KittiLabel()

    label.set_type(type)

    dimensions = calculate_dimensions(npc)
    if(type == "Pedestrian"):
        dimensions[1] = 2.5 * dimensions[1]
        dimensions[2] = 2.5 * dimensions[2]
    if(type=="Cyclist"):
        dimensions[0] = 1.5 * dimensions[0]
        dimensions[1] = 0.56 #corregir bug de extent.y = 0
    label.set_dimensions(dimensions)

    location = calculate_location(npc,world_2_camera,world_2_lidar,rot_trans_matrix)
    if(type == "Pedestrian"):
        location[1] = location[1] + dimensions[0]/2 
    label.set_location(location)

    safe_bbox_2D,bbox_2D = calculate_bbox2D(npc,K,world_2_camera,image_w,image_h)
    label.set_bbox(safe_bbox_2D)

    rotation_y=calculate_rotationy(npc,vehicle)
    label.set_rotation_y(rotation_y)

    alpha = calculate_alpha(rotation_y,location)
    label.set_alpha(alpha)

    truncated = calculate_truncated(bbox_2D, safe_bbox_2D)
    label.set_truncated(truncated)
    '''
    print(type)
    print(location)
    print(dimensions)
    print(safe_bbox_2D)
    print(bbox_2D)
    print(rotation_y)
    print(alpha)
    print(truncated)
    '''

    return label

def generate_kittilabel_tr(type_class, actor, lidar, world_2_camera,K,image_w,image_h,world_2_lidar,rot_trans_matrix):
    label = KittiLabel()

    label.set_type(type_class)

    dimensions = calculate_dimensions_bb(actor.bounding_box)
    label.set_dimensions(dimensions)

    location = calculate_location(actor,world_2_camera,world_2_lidar,rot_trans_matrix)
    label.set_location(location)

    safe_bbox_2D,bbox_2D = calculate_bbox2D(actor,K,world_2_camera,image_w,image_h)
    label.set_bbox(safe_bbox_2D)

    rotation_y=calculate_rotationy(actor,lidar)
    label.set_rotation_y(rotation_y)

    alpha = calculate_alpha(rotation_y,location)
    label.set_alpha(alpha)

    truncated = calculate_truncated(bbox_2D, safe_bbox_2D)
    label.set_truncated(truncated)
    '''
    print(type_class)
    print(location)
    print(dimensions)
    print(safe_bbox_2D)
    print(bbox_2D)
    print(rotation_y)
    print(alpha)
    print(truncated)
    '''

    return label

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

def generate_kittilabel_from_line(line):
    label = KittiLabel()

    label.set_type(get_label_type(line))

    label.set_location(get_label_location(line)) 

    label.set_dimensions(get_label_dimensions(line))

    label.set_bbox(get_label_bbox(line))

    label.set_rotation_y(get_label_rotation(line))

    label.set_alpha(get_label_alpha(line))

    label.set_truncated(get_label_truncated(line))

    label.set_occluded(get_label_occluded(line))

    return label
