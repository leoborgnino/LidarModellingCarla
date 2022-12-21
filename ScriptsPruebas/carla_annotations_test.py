import glob
import os
import sys
import argparse
import time
from datetime import datetime
import random
import numpy as np
from numpy import random
from matplotlib import cm
import open3d as o3d
from queue import Queue
from queue import Empty
import logging
import cv2

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

def build_projection_matrix(w, h, fov):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K

def get_image_point(loc, K, w2c):
        # Calculate 2D projection of 3D coordinate

        # Format the input coordinate (loc is a carla.Position object)
        point = np.array([loc.x, loc.y, loc.z, 1])
        # transform to camera coordinates
        point_camera = np.dot(w2c, point)
        #point_camera = np.dot(w2c, loc)

        # New we must change from UE4's coordinate system to an "standard"
        # (x, y ,z) -> (y, -z, x)
        # and we remove the fourth componebonent also
        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

        # now project 3D->2D using the camera matrix
        point_img = np.dot(K, point_camera)
        # normalize
        point_img[0] /= point_img[2]
        point_img[1] /= point_img[2]

        return point_img[0:2]

def create_images_directorie(current_datetime):
    images_folder = 'images'
    images_path = os.path.join(current_datetime,images_folder)
    os.makedirs(images_path)

    return images_path

def create_pointclouds_directorie(current_datetime):
    pointclouds_folder = 'point_clouds'
    pointclouds_path = os.path.join(current_datetime,pointclouds_folder)
    os.makedirs(pointclouds_path)

    return pointclouds_path

def create_labels_directorie(current_datetime):
    labels_folder = 'labels'
    labels_path = os.path.join(current_datetime,labels_folder)
    os.makedirs(labels_path)

    return labels_path

def generate_lidar_bp(blueprint_library,delta):
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('upper_fov', str(2.0))
    lidar_bp.set_attribute('lower_fov', str(-24.8))
    lidar_bp.set_attribute('channels', str(64))
    lidar_bp.set_attribute('range', str(120))
    lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
    lidar_bp.set_attribute('points_per_second', str(1300000))
    lidar_bp.set_attribute('noise_stddev', str(0.01))
    lidar_bp.set_attribute('dropoff_general_rate',str(0.0))
    #lidar_bp.set_attribute('sensor_tick', str(0.1)) #si es el doble q delta, va a dar un dato cada 2 ticks
    return lidar_bp

def generate_camera_bp(blueprint_library):
    camera_bp = blueprint_library.filter("sensor.camera.rgb")[0]
    camera_bp.set_attribute("image_size_x", str(1242)) #Resolucion de las imagenes de Kitti
    camera_bp.set_attribute("image_size_y", str(375))

    return camera_bp

def spawn_vehicles(spawn_points,number_of_vehicles,vehicles_bp,traffic_manager,client):
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor
    vehicles_list = []

    batch = []
    for n, transform in enumerate(spawn_points):
        if n >= number_of_vehicles:
            break
        blueprint = random.choice(vehicles_bp)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        blueprint.set_attribute('role_name', 'autopilot')

        batch.append(SpawnActor(blueprint, transform)
            .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

    for response in client.apply_batch_sync(batch, True):
        if response.error:
            logging.error(response.error)
        else:
            vehicles_list.append(response.actor_id)

    return vehicles_list

def save_image(images_path, image_data):
    image_path =  './%s/%.6d.png' % (images_path, image_data.frame)
    image_data.save_to_disk(image_path)
    #print('imagen %.6d.bin guardada' % image_data.frame)

def save_pointcloud(pointclouds_path,pointcloud):
    pc = np.copy(np.frombuffer(pointcloud.raw_data, dtype=np.dtype('f4')))
    pc = np.reshape(pc, (int(pc.shape[0] / 4), 4))
    pointcloud_path = './%s/%.6d.bin' % (pointclouds_path, pointcloud.frame) 
    pc.tofile(pointcloud_path)
    #print('point cloud %.6d.bin guardada' % lidar_data.frame)

def main(arg):
    """Spawnea el LIDAR HDL-64E y una camara RGB en un vehiculo, y genera un paso de simulacion"""
    #Cliente y simulador
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    
    #Crear directorios para guardar los datos, nubes de puntos e imagenes
    current_datetime = datetime.now().strftime("%d-%m-%y_%X")
    images_path = create_images_directorie(current_datetime)
    pointclouds_path = create_pointclouds_directorie(current_datetime)
    labels_path = create_labels_directorie(current_datetime)

    try:
        #Setea modo sincrono en la simulacion con delta fijo
        original_settings = world.get_settings()
        settings = world.get_settings()

        #traffic manager
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5) #distancia a mantener entre vehiculos
        #traffic_manager.set_hybrid_physics_mode(True) #desactiva las fisicas de los vehiculos lejanos al vehiculo hero, reduce el computo
        #traffic_manager.set_hybrid_physics_radius(100.0) #dentro de este radio, si se calculan las fisicas
        
        delta = 0.05
        #delta = 0.1
        settings.fixed_delta_seconds = delta
        settings.synchronous_mode = True
        world.apply_settings(settings)
        
        blueprint_library = world.get_blueprint_library()

        #Crea el LIDAR con las especificaciones de HDL-64E
        lidar_bp = generate_lidar_bp(blueprint_library,delta)

        #Crea la camara RGB
        camera_bp = generate_camera_bp(blueprint_library)

        #Crea vehiculo sobre el cual montar los sensores
        vehicle_bp = blueprint_library.filter("vehicle.lincoln.mkz_2017")[0]
        #vehicle_bp.set_attribute('role_name', 'hero')
        vehicle_bp.set_attribute('role_name', 'autopilot')

        #Spawnear vehiculo y sensores
        sp_vehicle = 0

        vehicle_transform = world.get_map().get_spawn_points()[sp_vehicle]
        vehicle = world.spawn_actor(
            blueprint=vehicle_bp,
            transform=vehicle_transform)
        vehicle.set_autopilot(True)

        #las coordenadas son relativas al vehiculo
        #x es el eje correspondiente a la direccion del auto, positivo seria hacia adelante
        #z es la altura
        camera = world.spawn_actor(blueprint=camera_bp,
            transform=carla.Transform(carla.Location(x=0.0, z=1.65)), #posicion segun Kitti 
            attach_to=vehicle)

        lidar = world.spawn_actor(
            blueprint=lidar_bp,
            transform=carla.Transform(carla.Location(x=-0.27, z=1.73)), #posicion segun Kitti
            attach_to=vehicle)

        #Funciones de callback para almacenar imagen y nube de puntos
        image_queue= Queue()
        lidar_queue= Queue()

        #Al recibir un nuevo data, se almacena en la cola
        lidar.listen(lambda data: sensor_callback(data,lidar_queue))
        camera.listen(lambda data: sensor_callback(data,image_queue))
        
        #spawnear trafico 

        vehicles_bp = blueprint_library.filter('vehicle.*') #blueprints de todos los vehiculos
        vehicles_bp = sorted(vehicles_bp, key=lambda bp: bp.id)
        spawn_points = world.get_map().get_spawn_points() #puntos de spawn del mapa en uso
        spawn_points.pop(sp_vehicle) #se quita el punto en el que se spawnea el vehivulo con los sensores
        random.shuffle(spawn_points) #mezclar 

        number_of_vehicles = 30

        vehicles_list = spawn_vehicles(spawn_points, number_of_vehicles, vehicles_bp, traffic_manager,client)

        #generar los ticks de simulacion necesarios hasta capturar los frames necesarios

        frames = arg.frames
        frames_captured = 0
        ticks = 0
                
        while frames_captured < frames:

            world.tick()
            ticks += 1
            time.sleep(0.005)

            #tras cada tick, en caso de haber dato disponible, se lo obtiene
            if not image_queue.empty():
                image = image_queue.get()
                
            if not lidar_queue.empty():
                pointcloud = lidar_queue.get()
            
            #cuando se tengan ambos datos (imagen y nube de puntos) de un mismo frame, se almacenan ambos
            #if ticks > 1 and (image.frame == pointcloud.frame):

                #guardar imagen
                #save_image(images_path,image)

                #guardar nube de puntos
                save_pointcloud(pointclouds_path,pointcloud)
                
                frames_captured = frames_captured + 1 

            sys.stdout.write("\r Capturados %d frames de %d en %d ticks" % (frames_captured,frames,ticks) + ' ')
            sys.stdout.flush()

            #Guardar anotacionees de los vehiculos
            annotation_path =  './%s/%.6d.txt' % (labels_path, pointcloud.frame)
            annotation_file = open(annotation_path,"w")

            #matriz de transformacion de coordenadas globales a lidar
            world_2_lidar = np.array(lidar.get_transform().get_inverse_matrix())

            #se obtienen todos los actores de la simulacion
            for npc in world.get_actors().filter('*vehicle*'):
                if npc.id != vehicle.id:
                    dist = npc.get_transform().location.distance(vehicle.get_transform().location)
                        
                        # Filter for the vehicles within 50m
                    if dist < 10:
                        #se obtiene la ubicacion del actor
                        npc_world_pos = npc.get_transform().location
                        #se agrega un 1.0 al punto para q sea de 4 y poder multiplicar por la matriz 4,4 de transformacion
                        npc_world_pos4 = np.array([npc_world_pos.x, npc_world_pos.y, npc_world_pos.z, 1])
                        #se transforma la coordenada
                        npc_lidar_pos = np.dot(world_2_lidar,npc_world_pos4)

                        #se escribe la coordenada en el acrchivo
                        annotation_file.write("x=%f,y=%f \n" % (npc_lidar_pos[0],npc_lidar_pos[1]))

            annotation_file.close()

    finally:
        world.apply_settings(original_settings)
        vehicle.destroy()
        lidar.destroy()
        camera.destroy()
        cv2.destroyAllWindows()

        #client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        time.sleep(0.5)

if __name__ == "__main__":
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '-f', '--frames',
        default=1000,
        type=int,
        help='cantidad de frames a capturar, default: 10')
    args = argparser.parse_args()

    try:
        main(args)
    except KeyboardInterrupt:
        print(' - Exited by user.')
