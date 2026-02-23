#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <AccelStepper.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3.h> 

// MAPEO DE PINES CNC SHIELD V3 a WEMOS D1 R32
#define EN_PIN   12 // Habilitador de la CNC Shield (Activo en LOW)

// Motor 1: Rotación de la cadera (Hip Yaw) -> Conectado al eje X de la Shield
#define HIP_YAW_STEP   26 
#define HIP_YAW_DIR    16 

// Motor 2: Elevación de la cadera (Hip Pitch) -> Conectado al eje Y de la Shield
#define HIP_PITCH_STEP 25 
#define HIP_PITCH_DIR  27 

// Motor 3: Flexión de la rodilla (Knee Pitch) -> Conectado al eje Z de la Shield
#define KNEE_PITCH_STEP 17 
#define KNEE_PITCH_DIR  14 

const float STEPS_PER_REV = 200.0; 

// Inicialización Motores
AccelStepper motorHipYaw(AccelStepper::DRIVER, HIP_YAW_STEP, HIP_YAW_DIR);
AccelStepper motorHipPitch(AccelStepper::DRIVER, HIP_PITCH_STEP, HIP_PITCH_DIR);
AccelStepper motorKnee(AccelStepper::DRIVER, KNEE_PITCH_STEP, KNEE_PITCH_DIR);

rcl_subscription_t subscriber;
geometry_msgs__msg__Vector3 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Callback, recibir y procesar los angulos 
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;
  
  // Mapeo del Vector3 (x, y, z) a las articulaciones físicas
  // msg->x = Ángulo de rotación de la cadera
  // msg->y = Ángulo de elevación de la cadera
  // msg->z = Ángulo de la rodilla

  long targetHipYaw   = (long)((msg->x / 360.0) * STEPS_PER_REV);
  long targetHipPitch = (long)((msg->y / 360.0) * STEPS_PER_REV);
  long targetKnee     = (long)((msg->z / 360.0) * STEPS_PER_REV);

  // Envio comandos de movimiento al ESP
  motorHipYaw.moveTo(targetHipYaw);
  motorHipPitch.moveTo(targetHipPitch);
  motorKnee.moveTo(targetKnee);
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // Encender la CNC Shield
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); 

  // Configuración dinámica (Para ajuste segun el peso, no se si sea necesario por los cicloidales pero lo incluyo mientras)
  motorHipYaw.setMaxSpeed(800);   motorHipYaw.setAcceleration(400);
  motorHipPitch.setMaxSpeed(800); motorHipPitch.setAcceleration(400);
  motorKnee.setMaxSpeed(800);     motorKnee.setAcceleration(400);

  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "neuro_leg_controller", "", &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "leg_joints_cmd"); // Tópico actualizado

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
  
  // Ejecución no bloqueante
  motorHipYaw.run();
  motorHipPitch.run();
  motorKnee.run();
}
