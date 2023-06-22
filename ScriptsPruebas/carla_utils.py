import carla
import random
import logging
import numpy as np

def generate_lidar_bp(blueprint_library,delta,ang_inc,material,reflectance_limit):
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('upper_fov', str(2.0))
    lidar_bp.set_attribute('lower_fov', str(-24.8))
    lidar_bp.set_attribute('channels', str(64))
    lidar_bp.set_attribute('range', str(130.0))
    lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
    lidar_bp.set_attribute('points_per_second', str(1333330))
    lidar_bp.set_attribute('noise_stddev', str(0.015))
    lidar_bp.set_attribute('dropoff_general_rate',str(0.05))
    lidar_bp.set_attribute('dropoff_intensity_limit',str(0.0))
    lidar_bp.set_attribute('dropoff_zero_intensity',str(0.0))
    #lidar_bp.set_attribute('sensor_tick', str(0.1)) #si es el doble q delta, va a dar un dato cada 2 ticks
    if(ang_inc):
        lidar_bp.set_attribute('model_angle', 'true')
    if(material):
        lidar_bp.set_attribute('model_material', 'true')
    lidar_bp.set_attribute('noise_stddev_intensity',str(0.05))
    lidar_bp.set_attribute('atmosphere_attenuation_rate',str(0.004))
    if(reflectance_limit):
        lidar_bp.set_attribute('model_reflectance_limits_function', 'true')
        lidar_bp.set_attribute('reflectance_limits_function_coeff_a', str(0.0005))
        lidar_bp.set_attribute('reflectance_limits_function_coeff_b', str(0.000054))
    return lidar_bp

def generate_lidar_bp_by_gui(blueprint_library,delta,lidar_all_configs):
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('upper_fov', lidar_all_configs[0])
    lidar_bp.set_attribute('lower_fov', lidar_all_configs[1])
    lidar_bp.set_attribute('channels', lidar_all_configs[2])
    lidar_bp.set_attribute('range', lidar_all_configs[3])
    lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
    lidar_bp.set_attribute('points_per_second', str(int(lidar_all_configs[4])/delta))
    lidar_bp.set_attribute('noise_stddev', lidar_all_configs[5])
    lidar_bp.set_attribute('noise_stddev_intensity',lidar_all_configs[6])
    lidar_bp.set_attribute('atmosphere_attenuation_rate',lidar_all_configs[7])
    lidar_bp.set_attribute('dropoff_general_rate', lidar_all_configs[8])
    lidar_bp.set_attribute('dropoff_intensity_limit', lidar_all_configs[9])
    lidar_bp.set_attribute('dropoff_zero_intensity',lidar_all_configs[10])
    #lidar_bp.set_attribute('sensor_tick', str(0.1)) #si es el doble q delta, va a dar un dato cada 2 ticks
    if(lidar_all_configs[11]):
        lidar_bp.set_attribute('model_angle', 'true')
    if(lidar_all_configs[12]):
        lidar_bp.set_attribute('model_material', 'true')
    if(lidar_all_configs[13]):
        lidar_bp.set_attribute('model_reflectance_limits_function', 'true')
        lidar_bp.set_attribute('reflectance_limits_function_coeff_a', lidar_all_configs[14])
        lidar_bp.set_attribute('reflectance_limits_function_coeff_b', lidar_all_configs[15])
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

def save_image(images_path, image_data,frame):
    image_path =  './%s/%.6d.png' % (images_path, frame)
    image_data.save_to_disk(image_path)
    #print('imagen %.6d.bin guardada' % image_data.frame)

def save_pointcloud(pointclouds_path,pointcloud,frame):
    pc = np.copy(np.frombuffer(pointcloud.raw_data, dtype=np.dtype('f4')))
    pc = np.reshape(pc, (int(pc.shape[0] / 4), 4))
    #UNREAL Y EL LIDAR INTERNO UTILIZA EL SISTEMA X: Foward, Y:Right, Z: Up
    #PERO EL LIDAR EN KITTI UTILIZA X: Foward, Y:Left, Z: Up
    #Por lo que hay que invertir las coordenas y

    pc[:,1] = -pc[:,1]
    pointcloud_path = './%s/%.6d.bin' % (pointclouds_path, frame) 
    pc.tofile(pointcloud_path)
    #print('point cloud %.6d.bin guardada' % lidar_data.frame)

def sensor_callback(data,queue):
    queue.put(data)

def spawn_pedestrians(seed,world,client,number_of_walkers,blueprintsWalkers):
    SpawnActor = carla.command.SpawnActor
    walkers_list = []
    all_id = []

    percentagePedestriansRunning = 0.0      # how many pedestrians will run
    percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
    if seed:
        world.set_pedestrians_seed(seed)
        random.seed(seed)
    # 1. take all the random locations to spawn
    spawn_points = []
    for i in range(number_of_walkers):
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        if (loc != None):
            spawn_point.location = loc
            spawn_points.append(spawn_point)
    # 2. we spawn the walker object
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        # set as not invincible
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        # set the max speed
        if walker_bp.has_attribute('speed'):
            if (random.random() > percentagePedestriansRunning):
                # walking
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                # running
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        else:
            print("Walker has no speed")
            walker_speed.append(0.0)
        batch.append(SpawnActor(walker_bp, spawn_point))
    results = client.apply_batch_sync(batch, True)
    walker_speed2 = []
    for i in range(len(results)):
        if results[i].error:
            pass
            logging.error(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})
            walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2
    #print(len(walkers_list))
    # 3. we spawn the walker controller
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list[i]["con"] = results[i].actor_id
    # 4. we put together the walkers and controllers id to get the objects from their id
    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
    all_actors = world.get_actors(all_id)

    # wait for a tick to ensure client receives the last transform of the walkers we have just created

    world.tick()

    # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
    # set how many pedestrians can cross the road
    world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
    for i in range(0, len(all_id), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        # max speed
        all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))
    
    return all_actors, all_id
    
