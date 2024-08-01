#include <micro_ros_arduino.h>  
#include <stdio.h>             
#include <rcl/rcl.h>           
#include <rcl/error_handling.h> 
#include <rclc/rclc.h>        
#include <rclc/executor.h>     
#include <std_msgs/msg/float32.h> 
#include <sensor_msgs/msg/imu.h>  
#include <sensor_msgs/msg/range.h>
#include <rosidl_runtime_c/string.h> 
#include "std_msgs/msg/detail/header__struct.h"
#include <std_msgs/msg/int32.h>

// Pines de sensor ultrasinico
const int pinTrig = 5; // disparo del sensor
const int pinEcho = 18; // eco del sensor

// Conversión de distancia
#define VELOCIDAD_SONIDO 0.034 
#define DISTANCIA_METROS 0.01 


rcl_publisher_t publicador_distancia; 
sensor_msgs__msg__Range msg_distancia; 
rosidl_runtime_c__String frame_id;
std_msgs__msg__Header header; 
rclc_executor_t ejecutor; 
rclc_support_t soporte; 
rcl_allocator_t asignador; 
rcl_node_t nodo; 
rcl_timer_t temporizador; 
rcl_subscription_t subscriber; 
std_msgs__msg__Int32 msg; 

// Pines de control de driver L9110s
const int a1a = 13;
const int a1b = 12; 
const int b1a = 14;
const int b1b = 27;

int velocidad = 85; // Valor de velocidad de 0 a 255

#define LED_PIN 13 // Pin para el LED que indica errores

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}} 
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
    delay(100); 
  }
}

// Función para leer la distancia del sensor 
float leerDistanciaUltrasonica() {
  long duracion; 
  float distanciaCm; 
  float distanciaMetros;

  // Pulso de disparo
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);

  // Lee la duracion del pulso de eco
  duracion = pulseIn(pinEcho, HIGH);
  distanciaCm = duracion * VELOCIDAD_SONIDO / 2; 
  distanciaMetros = distanciaCm * DISTANCIA_METROS;
  return distanciaMetros; 
}

void temporizador_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer != NULL) {
    float distancia = leerDistanciaUltrasonica(); 
    msg_distancia.range = distancia;
    RCSOFTCHECK(rcl_publish(&publicador_distancia, &msg_distancia, NULL)); 
  }
}

// Funcin callback suscripcion para manejo de motores del movil
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

void setup() {
  set_microros_transports(); 
  pinMode(LED_PIN, OUTPUT); 
  digitalWrite(LED_PIN, HIGH); 
  pinMode(pinTrig, OUTPUT); 
  pinMode(pinEcho, INPUT); 

  asignador = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&soporte, 0, NULL, &asignador));
  RCCHECK(rclc_node_init_default(&nodo, "esp32_valor_ultra", "", &soporte));


  RCCHECK(rclc_publisher_init_default(
    &publicador_distancia,
    &nodo,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "distancia_ultrasonica"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &nodo,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "diferencial_control_remoto"));

  const unsigned int temporizador_timeout = 1000; 
  RCCHECK(rclc_timer_init_default(
    &temporizador,
    &soporte,
    RCL_MS_TO_NS(temporizador_timeout),
    temporizador_callback));

  RCCHECK(rclc_executor_init(&ejecutor, &soporte.context, 2, &asignador));
  RCCHECK(rclc_executor_add_timer(&ejecutor, &temporizador));
  RCCHECK(rclc_executor_add_subscription(&ejecutor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  frame_id.data = "sensor_link";
  header.frame_id = frame_id;
  msg_distancia.header = header;
  msg_distancia.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  msg_distancia.field_of_view = 0.14; 
  msg_distancia.min_range = 0.02; 
  msg_distancia.max_range = 3.0; 
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&ejecutor, RCL_MS_TO_NS(100))); 
  delay(100); 
}

