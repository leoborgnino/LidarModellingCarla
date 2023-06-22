#funciones para leer, guardar, modificar, eliminar configuraciones de lidar desde un archivo json
import json

LIDAR_CONFIGS_PATH = './lidar_configs.json'

#leer la config con un determinado nombre
def lidar_config_to_array(lidar_config_name):
    with open(LIDAR_CONFIGS_PATH) as f:
        lidar_configs_json = json.load(f)
    
    lidar_config_array = []
    for lidar_config in lidar_configs_json['lidar_configs']:
        if lidar_config['name'] == lidar_config_name:
            for key,value in lidar_config.items():
                if key != 'name':
                    lidar_config_array.append(value)
    
    #print(lidar_config_array)
    return lidar_config_array

'''
#leer el archivo e iterar sobre el array de configs
def read_all_lidar_configs_json():
    with open(LIDAR_CONFIGS_PATH) as f:
        lidar_configs_json = json.load(f)
    
    lidar_configs_arrays=[]
    for lidar_config in lidar_configs_json['lidar_configs']:
        lidar_configs_arrays.append(lidar_config_to_array(lidar_config)) 
    
    return lidar_configs_arrays
'''

#leer el archivo y obtener nombres de las configs
def read_lidar_configs_names_json():
    with open(LIDAR_CONFIGS_PATH) as f:
        lidar_configs_json = json.load(f)
    
    lidar_configs_names=[]
    for lidar_config in lidar_configs_json['lidar_configs']:
        lidar_configs_names.append(lidar_config['name']) 
    
    return lidar_configs_names

#guardar una config
def save_lidar_config_json(lidar_name,lidar_config_array):
    lidar_config_dic = {            
        "name": lidar_name,
        "upper_fov": lidar_config_array[0],
        "lower_fov": lidar_config_array[1],
        "channels": lidar_config_array[2],
        "range": lidar_config_array[3],
        "points_per_rot": lidar_config_array[4],
        "noise_stddev": lidar_config_array[5],
        "noise_stddev_intensity": lidar_config_array[6],
        "atmosphere_attenuation_rate": lidar_config_array[7],
        "dropoff_general_rate": lidar_config_array[8],
        "dropoff_intensity_limit": lidar_config_array[9],
        "dropoff_zero_intensity": lidar_config_array[10],
        "model_ang_inc": lidar_config_array[11],
        "model_material" : lidar_config_array[12],
        "model_limit_func" : lidar_config_array[13],
        "limit_func_coeff_a": lidar_config_array[14],
        "limit_func_coeff_b": lidar_config_array[15]
    }

    with open(LIDAR_CONFIGS_PATH) as f:
        lidar_configs_json = json.load(f)

    #si ya existe se modifica, sino se agrega
    if check_existing_config_json(lidar_name):
        pass
        index_config=get_existing_config_index(lidar_name)
        lidar_configs_json['lidar_configs'][index_config] = lidar_config_dic
    else:
        lidar_configs_json['lidar_configs'].append(lidar_config_dic)

    with open(LIDAR_CONFIGS_PATH, 'w') as f:
        json.dump(lidar_configs_json, f)

#verificar si ya existe una config con ese nombre
def check_existing_config_json(lidar_config_name):
    with open(LIDAR_CONFIGS_PATH) as f:
        lidar_configs_json = json.load(f)

    for lidar_config in lidar_configs_json['lidar_configs']:
        if lidar_config['name'] == lidar_config_name:
            return True
    
    return False

def get_existing_config_index(lidar_config_name):
    with open(LIDAR_CONFIGS_PATH) as f:
        lidar_configs_json = json.load(f)

    for i,lidar_config in enumerate(lidar_configs_json['lidar_configs']):
        if lidar_config['name'] == lidar_config_name:
            return i

if __name__ == "__main__":

    #name='prueba'
    configs = ['20.0', '-24.8', '32', '120.0', '133333', '0.015', '0.05', '0.004', '0.0', '0.0', '0.0', True, True, True, '0.0005', '0.000054']
    #save_lidar_config_json(name,configs)

    print(check_existing_config_json('prueba'))
    print(get_existing_config_index('prueba'))
    save_lidar_config_json('prueba',configs)
    save_lidar_config_json('prueba2',configs)

    print(lidar_config_to_array('prueba'))
    print(lidar_config_to_array('prueba2'))

