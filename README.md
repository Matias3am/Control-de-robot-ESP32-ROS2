# PROGRAMACIÓN Y ENSAMBLE DE ROBOT DIFERENCIAL + VISUALIZACIÓN DE SENSOR:  ESP32 A RVIZ/ROS2 

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


# Programación ESP-32: Publisher (HC-04) / Subscriber (Motores)

```cpp
#include <micro_ros_arduino.h>  // Incluye la biblioteca de micro-ROS para Arduino
#include <stdio.h>             // Biblioteca estándar para funciones de entrada/salida
#include <rcl/rcl.h>           // Biblioteca de comunicación de ROS 2
#include <rcl/error_handling.h> // Biblioteca de manejo de errores de ROS 2
#include <rclc/rclc.h>         // Biblioteca para la inicialización de ROS 2 en C
#include <rclc/executor.h>     // Ejecutor para manejar la ejecución de nodos en ROS 2
#include <std_msgs/msg/float32.h> // Biblioteca de mensajes estándar para tipo float32
#include <sensor_msgs/msg/imu.h>  // Biblioteca de mensajes para IMU (unidad de medición inercial)
#include <sensor_msgs/msg/range.h>// Biblioteca de mensajes para rango de sensores
#include <rosidl_runtime_c/string.h> // Manejo de cadenas en ROS 2
#include "std_msgs/msg/detail/header__struct.h"
#include <std_msgs/msg/int32.h> // Biblioteca de mensajes estándar para tipo int32

// Pines utilizados para el sensor ultrasónico
const int pinTrig = 5; // Pin de disparo del sensor ultrasónico
const int pinEcho = 18; // Pin de eco del sensor ultrasónico

// Constantes para la conversión de distancia
#define VELOCIDAD_SONIDO 0.034 // Velocidad del sonido en cm/µs
#define DISTANCIA_METROS 0.01 // Conversión de cm a metros 

// Variables globales para la comunicación con ROS 2
rcl_publisher_t publicador_distancia; // Publicador para mensajes de distancia
sensor_msgs__msg__Range msg_distancia; // Mensaje para el rango del sensor
rosidl_runtime_c__String frame_id;
std_msgs__msg__Header header; 
rclc_executor_t ejecutor; // Ejecutor para manejar temporizadores y callbacks
rclc_support_t soporte; // Soporte para la inicialización de ROS 2
rcl_allocator_t asignador; // Asignador de memoria para ROS 2
rcl_node_t nodo; // Nodo de ROS 2
rcl_timer_t temporizador; // Temporizador para callbacks periódicos
rcl_subscription_t subscriber; // Suscriptor para recibir mensajes
std_msgs__msg__Int32 msg; // Mensaje para suscripción

// Variables para el control de motores en el ESP32
const int a1a = 13;
const int a1b = 12; 
const int b1a = 14;
const int b1b = 27;

int velocidad = 85; // Valor de velocidad de 0 a 255

#define LED_PIN 13 // Pin para el LED que indica errores

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}} // Macro para verificar el resultado de las funciones de ROS 2 y manejar errores
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Función que entra en un bucle infinito en caso de error, parpadeando el LED
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Cambia el estado del LED
    delay(100); // Espera 100 ms
  }
}

// Función para leer la distancia del sensor ultrasónico
float leerDistanciaUltrasonica() {
  long duracion; // Variable para almacenar la duración del pulso
  float distanciaCm; // Distancia en centímetros
  float distanciaMetros;

  // Genera el pulso de disparo
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);

  // Lee la duración del pulso de eco
  duracion = pulseIn(pinEcho, HIGH);
  distanciaCm = duracion * VELOCIDAD_SONIDO / 2; // Calcula la distancia en centímetros
  distanciaMetros = distanciaCm * DISTANCIA_METROS;
  return distanciaMetros; // Retorna la distancia en metros
}

// Función de callback que se ejecuta cada vez que el temporizador se activa
void temporizador_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer != NULL) {
    float distancia = leerDistanciaUltrasonica(); // Lee la distancia del sensor ultrasónico
    msg_distancia.range = distancia; // Asigna la distancia al mensaje
    RCSOFTCHECK(rcl_publish(&publicador_distancia, &msg_distancia, NULL)); // Publica el mensaje de distancia
  }
}

// Función de callback para la suscripción
void subscription_callback(const void * msgin) {  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg->data == 0){ 
    analogWrite(a1a, 0);
    analogWrite(a1b, 0); // 0 detener
    analogWrite(b1a, 0);
    analogWrite(b1b, 0);
  } 
  else if (msg -> data == 1) {
    analogWrite(a1a, velocidad);
    analogWrite(a1b, 0); // 1 para avanzar
    analogWrite(b1a, velocidad);
    analogWrite(b1b, 0);
  }
  else if (msg -> data == 2) {
    analogWrite(a1a, 0);
    analogWrite(a1b, velocidad); // 2 para retroceder
    analogWrite(b1a, 0);
    analogWrite(b1b, velocidad);
  }
  else if (msg -> data == 3 ){
    analogWrite(a1a, velocidad);
    analogWrite(a1b, 0); // 3 izquierda
    analogWrite(b1a, 0);
    analogWrite(b1b, 0);
  }
  else if (msg -> data == 4 ){
    analogWrite(a1a, 0);
    analogWrite(a1b, 0); // 4 derecha
    analogWrite(b1a, velocidad);
    analogWrite(b1b, 0);
  }
}

// Función de configuración inicial
void setup() {
  set_microros_transports(); // Inicializa los transportes de micro-ROS
  pinMode(LED_PIN, OUTPUT); // Configura el pin del LED como salida
  digitalWrite(LED_PIN, HIGH); // Enciende el LED
  pinMode(pinTrig, OUTPUT); // Configura el pin de disparo del sensor ultrasónico como salida
  pinMode(pinEcho, INPUT); // Configura el pin de eco del sensor ultrasónico como entrada

  // Inicializa el asignador y el soporte para ROS 2
  asignador = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&soporte, 0, NULL, &asignador));
  RCCHECK(rclc_node_init_default(&nodo, "esp32_valor_ultra", "", &soporte));

  // Inicializa el publicador para el mensaje de distancia
  RCCHECK(rclc_publisher_init_default(
    &publicador_distancia,
    &nodo,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "distancia_ultrasonica"));

  // Inicializa el suscriptor
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &nodo,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "diferencial_control_remoto"));

  // Inicializa el temporizador
  const unsigned int temporizador_timeout = 1000; // Tiempo de espera en milisegundos
  RCCHECK(rclc_timer_init_default(
    &temporizador,
    &soporte,
    RCL_MS_TO_NS(temporizador_timeout),
    temporizador_callback));

  // Inicializa el ejecutor y agrega el temporizador y suscriptor
  RCCHECK(rclc_executor_init(&ejecutor, &soporte.context, 2, &asignador));
  RCCHECK(rclc_executor_add_timer(&ejecutor, &temporizador));
  RCCHECK(rclc_executor_add_subscription(&ejecutor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // Configura el mensaje de distancia
  frame_id.data = "sensor_link";
  header.frame_id = frame_id;
  msg_distancia.header = header;
  msg_distancia.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  msg_distancia.field_of_view = 0.14; // Campo de visión en radianes
  msg_distancia.min_range = 0.02; // Rango mínimo en metros
  msg_distancia.max_range = 3.0; // Rango máximo en metros
}

// Función principal que se ejecuta en bucle
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&ejecutor, RCL_MS_TO_NS(100))); // Ejecuta el ejecutor
  delay(100); // Espera 100 ms
}

```

De aqui lo más importante a rescatar es:
``` python
Frame_id = " sensor_link "
Tópico_Publisher   = " distancia_ultrasonica "
Tópico_Subscriber = " diferencial_control_remoto "
```

# Ejecución del programa (ETAPA 1)
## 1.a- Comprobar puerto 

Si estamos utilizando maquinas virtuales para trabajar, por ejemplo VirtualBox (Mi caso) , tenemos que habilitar la lectura de puertos y para esto nos vamos a la seccion de configuración de la máquina como en la imagen de abajo y a la derecha de la ventana emerguente apareceran unos cables con simbolos encima, hay que presionar el que tenga el simbolo + de color verde y agregar el driver correspondiente al manejo de los puertos, en mi caso agregué el que aparece ahí, con esto listo debería reconocer el Ubuntu o sistema operativo que estes utilizando los puertos COM:

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/Configuraci%C3%B3n_COM.png)

Hecho lo anterior para poder asegurarse de que el ESP-32 esté conectado a nuestro ambiente de UBUNTU, una vez inicializado el sistema operativo tenemos que dirigirnos hacia el directorio /dev ya que aquí se encuentran todos los dispositivos hardware y puertos del sistema
``` console
cd /dev
ls 
```

Una vez ejecutados los comandos deberiamos ver algo como la imagen de abajo, si aparece el dispositivo ttyUSB0 significa que la conexión entre el ESP-32 y ubuntu se realizó de la manera correcta: 

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/comprobar_puerto.jpeg)

Si no te aparece el puerto ttyUSB0 prueba desconectando el cable del ESP-32 y volviendolo a conectar las veces que sean necesarios revisando constantemente con ls para verificar que aparezca el puerto, si estas haciendo esto por primera vez puede que sea necesario que tengas que reiniciar el computador para que empiece a funcionar todo como debería. 

## 1.b.- Crear e iniciar agente 
Para poder inicializar el programa cargado en nuestro ESP-32 tenemos que crear un agente en el sistema lo que hará que los valores sensorizados y compartidos por el micro-controlador lleguen al ubuntu, para lograr esto se tienen que ejecutar los siguientes comandos: 
``` console
# Instalación de dependencias
sudo apt install python3-rosdep
# Iniciar el entorno de ROS en la terminal
source /opt/ros/$ROS_DISTRO/setup.bash
# Crear y entrar a un directorio para actuar como agente (este se puede llamar como tú quieras)
mkdir uros_ws && cd uros_ws
# Instalamos un repositorio que es necesario para la ejecución
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup 
```
$ROS_DISTRO sería basicamente la distribución que estamos utilizando, en este caso yo usé ROS2 : Iron asi que habria que escribir iron solamente ahí 
``` console
# Incializamos las dependencias 
rosdep init / sudo rosdep init
# Actualizamos la base de datos de dependencias y luego instalamos
# todas las dependencias necesarias desde el directorio 'src'.
rosdep update && rosdep install --from-paths src --ignore-src -y
# Compilamos el espacio de trabajo utilizando 'colcon build',
colcon build 
# Configuramos el entorno de trabajo, asegurándonos de que los paquetes
# recién construidos estén disponibles para su uso.
source install/local_setup.bash
# Iniciamos el proceso de configuración de micro-ROS.
ros2 run micro_ros_setup create_agent_ws.sh 
# Construimos el agente con el script proporcionado, asegurando que
# todas las configuraciones necesarias estén en su lugar
ros2 run micro_ros_setup build_agent.sh 
# Volvemos a configurar el entorno para incluir las configuraciones del agente.
source install/local_setup.sh 
```
Aquí hay que hacer un break, ya que generalmente el puerto no tendrá los permisos de ejecución necesarios, para esto hay que ejecutar el siguiente comando para darle acceso al sistema: 
``` console
# Cambiamos los permisos del dispositivo para permitir el acceso completo.
sudo chmod -R 777 /dev/ttyUSB0
# Finalmente, iniciamos el agente de micro-ROS, que actuará como el
# intermediario entre el microcontrolador y el sistema principal.
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 
```
Si todo resultó de manera correcta debería verse como la siguiente imagen:

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/iniciar_agente1.jpeg)

### Cabe destacar que una vez que creamos un agente no es necesario tener que compilar todo de nuevo, cada vez que queramos conectarnos al agente solamente necesitamos ejecutar estas lineas:

``` console 
cd (carpeta del agente)
source install/local_setup.sh
sudo chmod -R 777 /dev/ttyUSB0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

En este punto podemos corroborar si se ejecutó bien el programa del ESP-32, entonces en una terminal aparte tenemos que ejecutar el siguiente comando:

``` console
ros2 topic list 
```

Se debería desplegar un mensaje parecido al siguiente: 

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/topic1.png)

Recordemos que nuestro topico en el código lo definimos como:

``` python
Tópico   = " distancia_ultrasonica "
```

Claramente no aparece, en este caso lo que debemos de hacer es un reset al ESP-32, entonces en la terminal en donde ejecutamos el puerto serial deberia verse así: 

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/iniciar_agente2.jpeg)

En el mensaje que aparece en el terminal podemos ver que  se creó el cliente junto al tópico, por lo que los datos deberían estar llegando al sistema operativo.

Volvemos a la terminal de tópicos para ingresar el comando ls nuevamente:

![](https://github.com/Matias3am/sensor_visualization-ROS-ESP32/blob/main/imagenes/topicos_s_p.png)

Aquí ya aparece nuestro tópico, para comprobar que le estén llegando los valores de los sensores tenemos que ingresar el siguiente comando en la terminal 

``` console
ros2 topic echo /{nombre de nuestro tópico}
# En mi caso sería : distancia_ultrasonica
```

Ingresando este comando la terminal debería entregar esto: 

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/valores_sensores.jpeg)

Si se visualiza el mensaje igual que la imagen anterior significa que se estan recepcionando de manera correcta la información desde el ESP-32

## 1.c.- Iniciar entorno de visualización 

Para poder visualizar los datos de mi sensor, necesitamos tener un archivo URDF a quien asociarle el sensor, en mi caso yo hice un robot diferencial de 2 ruedas, junto con una caja al frente de este: 

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/robot.jpeg)

En el código del ESP-32 definimos un frameid:

``` python
Frame_id = " sensor_link "
```

Este nombre es importante ya que en el caso del sensor ultrasónico tenemos que vincular en algún cuerpo del robot el sensor, en mi caso vincule el sensor ultra sónico a la cajita que se puede ver al frente del robot de la siguiente manera:

```urdf
    <link name="sensor_link">
        <visual>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <geometry>
                <box size="0.04 0.04 0.04"/>
            </geometry>
            <material name="sensor_link_color">
                <color rgba="0.5 0.6 0.0 1.0"/>
            </material>
        </visual>
        <collision>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <geometry>
                <box size="0.04 0.04 0.04"/>
            </geometry>
        </collision>
    </link>
```

Con el sensor vinculado al robot, podemos ejecutar el siguiente comando para abrir el rviz2 con el modelo de nuestro robot: 

```terminal
ros2 launch urdf_tutorial display.launch.py model:=$HOME/{dirección hacia nuestro robot urdf}
```

Lo que se debería ver seria nuestro robot sin más, para poder visualizar el sensor ultra sónico en el rviz lo que tenemos que hacer es presionar el boton add en la esquina inferior izquierda y agregar la opción de range:

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/config_rviz.jpeg)

Hecho esto agregamos el nombre del "Tópico" , no del frameid 

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/rviz_range.jpeg)

Resultando en lo siguiente: 

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/ambiente.jpeg)


# Ejecución con launcher 
Hechos todos los pasos anteriores, lo que hice yo fue crear un "package" en donde configuré un launcher para ejecutar la visualización con las configuraciones ya hechas de rviz y el robot, en vez de utilizar el comando  

``` console
ros2 launch urdf_tutorial display.launch.py model:=$HOME/{dirección hacia nuestro robot urdf}
```

Solamente nos vamos a la carpeta de sensores que sería la principal, realizamos:

``` console
colcon build
# Luego
source install/setup.bash
```

Para ingresar el comando de la imagen de abajo:

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/launcher_comando.jpeg)

Debería visualizarse el mismo ambiente anterior: 

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/ambiente.jpeg)

# Control del movimiento del robot (Python) (ETAPA 2)
# 2.a- Conexión Publisher -> Subscriber -> Robot

Para poder controlar el robot mediante la terminal se creó un launchfile de Python el cual será el encargado de leer valores enteros en la terminal y enviar estos valores al tópico subscriber del ESP-32, para esto es necesario crear una carpeta aparte en donde tendremos nuestro programa de Ros Python. 

``` console
# Comando para crear un paquete Python
ros2 pkg create --build-type ament_python {Nombre del ambiente de trabajo}
```

El paquete creado debería generarte un entorno parecido a lo siguiente: 

![](https://github.com/Matias3am/sensor_visualization-ROS-ESP32/blob/main/imagenes/paquete_ejemplo.png)

En donde la primera carpeta que se llamará igual que el nombre del paquete que asignamos contendrá los programas principales que deseemos agregar, las demás carpetas no tienen implicancia en el programa, excepto por el archivo "setup.py" el cual se verá de la siguiente forma: 

``` python
from setuptools import find_packages, setup

package_name = '{Nombre del paquete}'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='{Nombre de usuario root de tú maquina virtual}',
    maintainer_email='{Email del usuario root}',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

De aqui lo que configuraremos será el entry_points, ya que esto definirá que programa queremos ejecutar en este entorno de trabajo

``` python
    entry_points={
        'console_scripts': [
            '{Cómo quieres que se llame el comando} = {nombre del paquete}.{programa que quieres ejecutar}:{función del programa que quieres ejecutar}'
        ],
```

Imaginemos que creamos un paquete llamado "procesamiento_Datos" en el cual tenemos un archivo "programa.py" con cierta lógica para publicar / procesar o subscribirse a datos en Ros, para poder ejecutar el programa tendrías que configurar el entry_point de la siguiente manera: 

``` python
    entry_points={
        'console_scripts': [
            ' iniciar_procesamiento = procesamiento_Datos.programa:main'
        ],
```

Aquí lo que hicimos es decirle al paquete que queremos correr la función "main" de "programa.py" que está en la carpeta "procesamiento_Datos", pero que al ejecutar el programa en terminal tenga que escribir "iniciar_procesamiento" para que empiece a hacer lo que queramos. 

Para compilar y ejecutar el programa tenemos que ir a la carpeta principal que hicimos para el paquete y escribir en la terminal los siguientes comandos: 

``` console
colcon build --packages-select {nombre del paquete}
source install/setup.bash
ros2 run {nombre del paquete} {nombre que definimos para ejecutar el programa}
```

Siguiendo el mismo ejemplo del caso anterior sería algo así:

``` console
colcon build --packages-select procesamiento_Datos
source install/setup.bash
ros2 run procesamiento_Datos iniciar_procesamiento
```

En nuestro caso, tenemos un programa en la carpeta "subscriptor.py" que se llama publicador.py el cual es un publisher que se encargará de enviar instrucciones a nuestro robot mediante lo que uno ingrese a la terminal

``` python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import threading
import time 
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Int32, 'diferencial_control_remoto', 10)

    def publish_message(self, data):
        msg = Int32()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    def spin_thread():
        rclpy.spin(minimal_publisher)

    thread = threading.Thread(target=spin_thread)
    thread.start()

    try:
        while True:
            command = input("0: Detener, 1: Frente, 2: Retroceso, 3: Izquierda, 4: Derecha, 5: Secuencia, 6: break = ")
            if command == "5":
                minimal_publisher.publish_message(1)
                time.sleep(4)
                minimal_publisher.publish_message(2)
                time.sleep(4)
                minimal_publisher.publish_message(3)
                time.sleep(4)
                minimal_publisher.publish_message(4)
                time.sleep(4)
                minimal_publisher.publish_message(0)
                time.sleep(4)
            elif command == "0":
                minimal_publisher.publish_message(0)
            elif command == "1":
                minimal_publisher.publish_message(1)
            elif command == "2":
                minimal_publisher.publish_message(2)
            elif command == "3":
                minimal_publisher.publish_message(3)
            elif command == "4":
                minimal_publisher.publish_message(4)
            elif command == "6":
                break
            else:
                print("Invalid command. Please enter 0, 1, 2, 3, 4 ,5 or 6")
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()
        thread.join()

if __name__ == '_main_':
    main()
```

Recordemos que nuestro ESP-32 al ejecutar el agente creará 2 topicos: 

``` python
Tópico_Publisher   = " distancia_ultrasonica "
Tópico_Subscriber = " diferencial_control_remoto "
```

En donde el primero es lo que envia el ESP-32 con respecto a las mediciones del sensor ultra sónico y el tópico "diferencial_control_remoto" es el subscriber al cual nosotros le estamos enviando comandos mediante el script de python que creamos. 

![](https://github.com/Matias3am/sensor_visualization-ROS-ESP32/blob/main/imagenes/control_python.jpeg)

Estos valores que se están enviando por la terminal son los que recibe el ESP-32 para darle control a los motores como se visualiza en el código: 

``` cpp
// Función de callback para la suscripción
void subscription_callback(const void * msgin) {  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg->data == 0){ 
    analogWrite(a1a, 0);
    analogWrite(a1b, 0); // 0 detener
    analogWrite(b1a, 0);
    analogWrite(b1b, 0);
  } 
  else if (msg -> data == 1) {
    analogWrite(a1a, velocidad);
    analogWrite(a1b, 0); // 1 para avanzar
    analogWrite(b1a, velocidad);
    analogWrite(b1b, 0);
  }
  else if (msg -> data == 2) {
    analogWrite(a1a, 0);
    analogWrite(a1b, velocidad); // 2 para retroceder
    analogWrite(b1a, 0);
    analogWrite(b1b, velocidad);
  }
  else if (msg -> data == 3 ){
    analogWrite(a1a, velocidad);
    analogWrite(a1b, 0); // 3 izquierda
    analogWrite(b1a, 0);
    analogWrite(b1b, 0);
  }
  else if (msg -> data == 4 ){
    analogWrite(a1a, 0);
    analogWrite(a1b, 0); // 4 derecha
    analogWrite(b1a, velocidad);
    analogWrite(b1b, 0);
  }
}
```

# Integración de todo (ETAPA 3)
# 3.a- Definición de componentes 
- ESP32 (Modelo: Wroom 32e )
- Sensor ultra-sónico Hc-04
- Motores Dc N20 (5v)
- Modelo 3D robot 
- Protoboard (Ensayo)
- Conversor corriente electrica -> 5V
  

# 3.b- Conexionado electrico de los componentes
![](https://github.com/Matias3am/sensor_visualization-ROS-ESP32/blob/main/conexionado_electrico.png)

# 3.c- Puesta en marcha
![](https://github.com/Matias3am/sensor_visualization-ROS-ESP32/blob/main/imagenes/robotcin.jpg)
