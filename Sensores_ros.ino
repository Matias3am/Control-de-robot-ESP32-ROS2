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

