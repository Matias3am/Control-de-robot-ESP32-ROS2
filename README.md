# VISUALIZACIÓN DE DATOS SENSORIALES DESDE ESP32 A RVIZ/ROS2 
```
Asignatura: Programación de robots con ROS

Integrantes: Matias Gonzalez Belmar, Pedro Díaz Herrera, Matias Marin Morales

Profesor: Sebastián Guajardo
```
![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/modelo.jpeg)

## _**REQUISITOS MÍNIMOS GENERALES**_ 
* Python
* ROS2 / IRON
* RVIZ
* Esp32
* Sensor HC-04
* Libreria de micro-ros


# Estructura de los archivos
* **launcher:** Aquí se encuentran los launchers necesarios para ejecutar la visualización en rviz.
  * **display_sensores.py:** Este archivo despliega la visualización del robot urdf junto al sensor. 
* **rviz:** Aquí se almacena la configuración de rviz para que al abrirse se visualice el sensor.
* **urdf:** Aquí se encuentran los archivos que describen la estructura del robot:
  * **robot_sensores.urdf:** Este archivo corresponde al modelo del robot junto al cuadrado que representará el sensor de ultra-sonido 

# Ejecución del programa 
## 1.- Comprobar puerto 

Si estamos utilizando maquinas virtuales para trabajar, por ejemplo VirtualBox (Mi caso) , tenemos que habilitar la lectura de puertos y para esto nos vamos a la seccion de configuración de la máquina como en la imagen de abajo y a la derecha de la ventana emerguente apareceran unos cables con simbolos encima, hay que presionar el que tenga el simbolo + de color verde y agregar el driver correspondiente al manejo de los puertos, en mi caso agregué el que aparece ahí, con esto listo debería reconocer el Ubuntu o sistema operativo que estes utilizando los puertos COM:

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/Configuraci%C3%B3n_COM.png)

Hecho lo anterior para poder asegurarse de que el ESP-32 esté conectado a nuestro ambiente de UBUNTU, primero tenemos que dirigirnos hacia el directorio /dev ya que aquí se encuentran todos los dispositivos hardware y puertos del sistema
```
cd /dev
ls 
```
Una vez inicializados los comandos deberiamos ver algo como la imagen de abajo, si aparece el dispositivo ttyUSB0 significa que la conexión entre el ESP32 y ubuntu se realizó de la manera correcta: 

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/comprobar_puerto.jpeg)

## 2.- Crear e iniciar agente 
Para poder inicializar el programa cargado en nuestro ESP-32 tenemos que crear un agente en el sistema lo que hará que los valores sensorizados y compartidos por el micro-controlador lleguen al ubuntu, para lograr esto se tienen que ejecutar los siguientes comandos: 
```
sudo apt install python3-rosdep  // Instalación de dependencias
source /opt/ros/$ROS_DISTRO/setup.bash // Iniciar el entorno de ros en la terminal
mkdir uros_ws && cd uros_ws // Creamos y entramos a un direcctorio para actuar como agente (Este se puede llamar como tú quieras)
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup // instalamos un repositorio que es necesario para la ejecución
```
$ROS_DISTRO sería basicamente la distribución que estamos utilizando, en este caso yo usé ROS2 : Iron asi que habria que escribir iron solamente ahí 
```
rosdep init / sudo rosdep init // incializamos las dependencias 
rosdep update && rosdep install --from-paths src --ignore-src -y // actualizamos e instalamos archivos descargados
colcon build // Inicializamos el ambiente  
source install/local_setup.bash // Importamos ros al ambiente de trabajo 
ros2 run micro_ros_setup // Iniciamos el setup
create_agent_ws.sh // Creamos el agente 
ros2 run micro_ros_setup build_agent.sh // Ejecutamos el setup para el agente  
source install/local_setup.sh // Importamos ros al ambiente del agente 
ros2 run micro_ros_agent micro_ros_agent // Iniciamos el agente 
```
Aquí hay que hacer un break, ya que generalmente el puerto no tendrá los permisos de ejecución necesarios, para esto hay que ejecutar el siguiente comando para darle acceso al sistema: 
```
sudo chmod -R 777 /dev/ttyUSB0
serial --dev /dev/ttyUSB0 // Inicializamos la conexión al puerto del ESP-32
```
Si todo resultó de manera correcta debería verse como la siguiente imagen:

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/iniciar_agente1.jpeg)

En este punto podemos corroborar si se ejecutó bien el programa del ESP-32, entonces en una terminal aparte tenemos que ejecutar el siguiente comando:

```
ros2 topic list 
```

Si no aparece el tópico del mensaje tenemos que hacer un reset al ESP-32, entonces en la terminal en donde ejecutamos el puerto serial deberia verse así: 

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/iniciar_agente2.jpeg)

En el mensaje que aparece en el terminal podemos ver que  se creó el cliente junto al tópico, por lo que los datos deberían estar llegando al sistema operativo.

## 3.- Iniciar entorno de visualización 

Con nuestro agente recibiendo información del ESP-32, solamente queda ejecutar el launcher de rviz como en la siguiente imagen:

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/launcher_comando.jpeg)

Resultando en lo siguiente: 

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/ambiente.jpeg)
