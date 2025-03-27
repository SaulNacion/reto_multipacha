# Reto Multipacha

## Descripción
Este repositorio es para documentar avances, problemas y resolución de estos para completar el Reto Multipacha.

## Desarrollo para ROS
### Creación del workspace
Se crea una carpeta **reto** para almacenar el workspace.
```bash
mkdir reto
```
Y dentro se crea el workspace **multipacha_ws** además se crea la carpeta **src** que se usará para almacenar el paquete pedido.
```bash
mkdir multipacha_ws
mkdir src
```
Se construye el workspace usando `colcon build`
y con `ls` se pueden ver los archivos generados.
```
build  install  log  src
``` 
Para crear el paquete **reto_m6** se ingresa a la carpeta **src** creada anteriormente.
```bash
cd src/
```
Una vez dentro se crea el paquete.
```
ros2 pkg create reto_m6 --build-type ament_python --dependencies rclpy rviz2
```
Si bien se pide usar las dependencias **rviz** y **rospy**  las cuales son de *ROS*, pero en este caso se está usando *ROS 2*.

Finalmente se ingresa al paquete creado mediante
```bash
cd reto_m6/
```
Y se crean los directorios **launch** y **urdf**
```bash
mkdir launch
mkdir urdf
```
