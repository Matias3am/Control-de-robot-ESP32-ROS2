# VISUALIZACIÓN DE DATOS SENSORIALES DESDE ESP32 A RVIZ/ROS2 

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


# Programación del micro-controlador ESP-32 + sensor Hc-04:

```cpp
#include <micro_ros_arduino.h>  // Incluye la biblioteca para micro-ROS en Arduino
// Bibliotecas necesarias para micro-ROS
#include <stdio.h>             // Biblioteca estándar para funciones de entrada/salida
#include <rcl/rcl.h>           // Biblioteca para la comunicación de ROS 2
#include <rcl/error_handling.h> // Biblioteca para el manejo de errores de ROS 2
#include <rclc/rclc.h>         // Biblioteca para la inicialización de ROS 2 en C
#include <rclc/executor.h>     // Biblioteca para la ejecución de nodos en ROS 2
#include <std_msgs/msg/float32.h> // Biblioteca para mensajes estándar de tipo float32
#include <sensor_msgs/msg/range.h>// Biblioteca para mensajes de rango de sensor
#include <rosidl_runtime_c/string.h> // Biblioteca para manejo de cadenas en ROS 2
#include "std_msgs/msg/detail/header__struct.h" // Importamos la estructura para el header del mensaje

// Pines utilizados para el sensor ultrasónico
const int pinTrig = 5; // Pin de disparo del sensor ultrasónico
const int pinEcho = 18; // Pin de eco del sensor ultrasónico

// Constantes para la conversión de distancia
#define VELOCIDAD_SONIDO 0.034 // Velocidad del sonido en cm/µs
#define distancia_Metros 0.01 // conversión cm a metros 

// Variables globales para la comunicación con ROS 2
rcl_publisher_t publicador_distancia; // Publicador para el mensaje de distancia
sensor_msgs__msg__Range msg_distancia; // Mensaje para el rango del sensor
rosidl_runtime_c__String frame_id;
std_msgs__msg__Header header; 
rclc_executor_t ejecutor; // Ejecutador para manejar los temporizadores y callbacks
rclc_support_t soporte; // Soporte para inicialización de ROS 2
rcl_allocator_t asignador; // Asignador de memoria para ROS 2
rcl_node_t nodo; // Nodo de ROS 2
rcl_timer_t temporizador; // Temporizador para ejecutar callbacks periódicos

#define LED_PIN 13 // Pin del LED para indicar errores

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
  distanciaMetros = distanciaCm * distancia_Metros;
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

  // Inicializa el temporizador
  const unsigned int temporizador_timeout = 1000; // Tiempo de espera en milisegundos
  RCCHECK(rclc_timer_init_default(
    &temporizador,
    &soporte,
    RCL_MS_TO_NS(temporizador_timeout),
    temporizador_callback));

  // Inicializa el ejecutor y agrega el temporizador
  RCCHECK(rclc_executor_init(&ejecutor, &soporte.context, 1, &asignador));
  RCCHECK(rclc_executor_add_timer(&ejecutor, &temporizador));

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
Tópico   = " distancia_ultrasonica "
```

# Ejecución del programa 
## 1.- Comprobar puerto 

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

## 2.- Crear e iniciar agente 
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
ros2 run micro_ros_setup
# Creamos el espacio de trabajo del agente utilizando un script de configuración.
create_agent_ws.sh 
# Construimos el agente con el script proporcionado, asegurando que
# todas las configuraciones necesarias estén en su lugar
ros2 run micro_ros_setup build_agent.sh 
# Volvemos a configurar el entorno para incluir las configuraciones del agente.
source install/local_setup.sh 
# Finalmente, iniciamos el agente de micro-ROS, que actuará como el
# intermediario entre el microcontrolador y el sistema principal.
ros2 run micro_ros_agent micro_ros_agent 
```
Aquí hay que hacer un break, ya que generalmente el puerto no tendrá los permisos de ejecución necesarios, para esto hay que ejecutar el siguiente comando para darle acceso al sistema: 
``` console
# Cambiamos los permisos del dispositivo para permitir el acceso completo.
sudo chmod -R 777 /dev/ttyUSB0
# Iniciamos la conexión al puerto del ESP-32 'ttyUSB0'
serial --dev /dev/ttyUSB0 
```
Si todo resultó de manera correcta debería verse como la siguiente imagen:

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/iniciar_agente1.jpeg)

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

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/topic_list.jpeg)

Aquí ya aparece nuestro tópico, para comprobar que le estén llegando los valores de los sensores tenemos que ingresar el siguiente comando en la terminal 

``` console
ros2 topic echo /{nombre de nuestro tópico}
# En mi caso sería : distancia_ultrasonica
```

Ingresando este comando la terminal debería entregar esto: 

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/valores_sensores.jpeg)

Si se visualiza el mensaje igual que la imagen anterior significa que se estan recepcionando de manera correcta la información desde el ESP-32

## 3.- Iniciar entorno de visualización 

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

```
ros2 launch urdf_tutorial display.launch.py model:=$HOME/{dirección hacia nuestro robot urdf}
```

Solamente nos vamos a la carpeta de sensores que sería la principal, realizamos:

```
colcon build
# Luego
source install/setup.bash
```

Para ingresar el comando de la imagen de abajo:

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/launcher_comando.jpeg)

Debería visualizarse el mismo ambiente anterior: 

![](https://github.com/Matias3am/sensor_visualization-ros2_rviz/blob/main/imagenes/ambiente.jpeg)

