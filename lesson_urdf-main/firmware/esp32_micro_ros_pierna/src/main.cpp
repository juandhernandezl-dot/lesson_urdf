#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <AccelStepper.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3.h>

// --- PINES LOCALES (CNC Shield V3) ---
// Eje X (joint_p / Hip Yaw)
#define HIP_YAW_STEP   26
#define HIP_YAW_DIR    16

// Eje Z (joint_r / Knee Pitch)
#define KNEE_PITCH_STEP 17
#define KNEE_PITCH_DIR  14

// --- CONFIGURACIÓN DE UART (Hacia ATmega2560) ---
// GPIO 4 corresponde al pin "Hold" en la CNC Shield V3
static const int UART_TX_PIN = 25;   
static const int UART_RX_PIN = 13; // No se conecta físicamente por seguridad (5V)
static const uint32_t UART_BAUD = 115200;

// Configuración de Fuerza y Velocidad (Modo Tractor)
const float STEPS_PER_REV = 200.0;
float velocidad_constante = 100.0; // Pasos por segundo (10ms de pausa)

// Instancias de los motores locales
AccelStepper motorHipYaw(AccelStepper::DRIVER, HIP_YAW_STEP, HIP_YAW_DIR);
AccelStepper motorKnee(AccelStepper::DRIVER, KNEE_PITCH_STEP, KNEE_PITCH_DIR);

// Variables de micro-ROS
rcl_subscription_t subscriber;
geometry_msgs__msg__Vector3 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Función de seguridad para limitar ángulos (-90° a 90°)
static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Función que transmite el ángulo Y (joint_c) al Mega
static void send_joint_c_deg(float hipPitchDeg) {
  Serial2.print("C:");
  Serial2.print(hipPitchDeg, 2);
  Serial2.print('\n'); // El terminador clave para que el Mega sepa que terminó de leer
}

// Callback al recibir coordenadas de MoveIt!
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Vector3 * m = (const geometry_msgs__msg__Vector3 *)msgin;

  // 1. Filtrar los ángulos por seguridad
  float hipYawDeg   = clampf(m->x, -90.0f, 90.0f);  // X: Local
  float hipPitchDeg = clampf(m->y, -90.0f, 90.0f);  // Y: Externo
  float kneeDeg     = clampf(m->z, -90.0f, 90.0f);  // Z: Local

  // 2. ENVIAR ORDEN AL ATMEGA2560 INMEDIATAMENTE
  send_joint_c_deg(hipPitchDeg);

  // 3. Procesar los motores locales
  long targetHipYaw = (long)((hipYawDeg / 360.0f) * STEPS_PER_REV);
  long targetKnee   = (long)((kneeDeg   / 360.0f) * STEPS_PER_REV);

  motorHipYaw.moveTo(targetHipYaw);
  motorKnee.moveTo(targetKnee);

  // 4. Configurar la dirección del Torque Constante
  motorHipYaw.setSpeed(targetHipYaw > motorHipYaw.currentPosition() ? velocidad_constante : -velocidad_constante);
  motorKnee.setSpeed(targetKnee > motorKnee.currentPosition() ? velocidad_constante : -velocidad_constante);
}

void setup() {
  // Comunicación USB principal para micro-ROS
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // Iniciar el puerto esclavo hacia el ATmega2560
  // Solo TX emitirá señal por el pin Hold de la CNC Shield
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // Limitar velocidad de los motores locales
  motorHipYaw.setMaxSpeed(500);
  motorKnee.setMaxSpeed(500);

  delay(2000);

  // Inicializar el nodo micro-ROS
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "neuro_leg_controller", "", &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "leg_joints_cmd"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  // 1. Escuchar a MoveIt!
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));

  // 2. Mover los motores locales X y Z con fuerza bruta y sin rampas
  motorHipYaw.runSpeedToPosition();
  motorKnee.runSpeedToPosition();
}