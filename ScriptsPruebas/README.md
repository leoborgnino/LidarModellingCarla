## Descripcion de funcionalidades de scripts de pruebas utilizando CARLA

- **carla_pointcloud.py**: sensor LiDAR HDL-64E, genera y guarda una nube de puntos en formato binario.
- **carla_pointcloud_image.py**: sensor LiDAR HDL-64E y camara RGB, montados sobre un vehiculo,  genera y guarda una nube de puntos en formato binario y una imgagen en formato png.
- **carla_pointcloud_image_route.py**: sensor LiDAR HDL-64E y camara RGB, montados sobre un vehiculo el cual recorre la ciudad,  genera y guarda nubes de puntos en formato binario e imagenes en formato png correspondidas. Con el argumento -f o --frames se puede especificar la cantidad de nubes e imagenes a capturar.