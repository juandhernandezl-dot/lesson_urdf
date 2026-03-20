#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <AccelStepper.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3.h>

// =========================
// CNC Shield V3 + DRV8825 (Wemos ESP32)
// Motores locales (NEMA17): joint_p (X) y joint_r (Z)
// Motor externo (NEMA23): joint_c lo mueve el Mega por UART
// =========================

// --- PINES LOCALES (CNC Shield V3) ---
// Eje X (joint_p / Hip Yaw)
#define HIP_YAW_STEP   26
#define HIP_YAW_DIR    16

// Eje Z (joint_r / Knee Pitch)
#define KNEE_PITCH_STEP 17
#define KNEE_PITCH_DIR  14

// --- UART hacia Arduino Mega (esclavo) ---
// IMPORTANTE: UART_TX_PIN es el GPIO real donde conectaste el cable al MEGA RX1(19)
static const int UART_TX_PIN = 25;     // <-- Tu TX hacia el Mega (ajusta si tu cable está en otro GPIO)
static const int UART_RX_PIN = 13;     // No conectar físicamente (Mega TX=5V)
static const uint32_t UART_BAUD = 115200;

// =========================
// PASOS: NEMA17 + DRV8825 (microstepping por jumpers)
// DRV8825 1/8 -> MS1=1, MS2=1, MS3=0 (normalmente 2 jumpers puestos)
// =========================
static constexpr float STEPS_PER_REV = 200.0f;      // NEMA17 típico
static constexpr float MICROSTEPS_LOCAL = 1.0f;     // DRV8825 en 1/8 (por jumpers)
static constexpr float GEAR_RATIO_LOCAL = 11.0f;     // si hay reducción en estos ejes

static constexpr float EFFECTIVE_STEPS_PER_REV_LOCAL =
    STEPS_PER_REV * MICROSTEPS_LOCAL * GEAR_RATIO_LOCAL;

// =========================
// MODO "TRACTOR" (torque): velocidad constante en pasos/seg
// Con microstep 1/8, 400-800 pasos/s suele ser buen inicio.
// =========================
static float velocidad_constante = 400.0f;  // pasos/seg (ajusta)

// =========================
// Motores locales
// =========================
AccelStepper motorHipYaw(AccelStepper::DRIVER, HIP_YAW_STEP, HIP_YAW_DIR);
AccelStepper motorKnee(AccelStepper::DRIVER, KNEE_PITCH_STEP, KNEE_PITCH_DIR);

// =========================
// micro-ROS
// =========================
rcl_subscription_t subscriber;
geometry_msgs__msg__Vector3 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// =========================
// Utilidades
// =========================
static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Enviar joint_c (Y) al Mega: "C:<deg>\n"
static void send_joint_c_deg(float hipPitchDeg) {
  Serial2.print("C:");
  Serial2.print(hipPitchDeg, 2);
  Serial2.print('\n');
}

// Callback al recibir leg_joints_cmd (x,y,z en grados)
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Vector3 * m = (const geometry_msgs__msg__Vector3 *)msgin;

  // 1) Limitar ángulos por seguridad
  float hipYawDeg   = clampf(m->x, -90.0f, 90.0f);  // joint_p (local)
  float hipPitchDeg = clampf(m->y, -90.0f, 90.0f);  // joint_c (Mega)
  float kneeDeg     = clampf(m->z, -90.0f, 90.0f);  // joint_r (local)

  // 2) Enviar Y al Mega inmediatamente
  send_joint_c_deg(hipPitchDeg);

  // 3) Convertir grados -> pasos (teniendo en cuenta microstep 1/8 del DRV8825)
  long targetHipYaw = (long)((hipYawDeg / 360.0f) * EFFECTIVE_STEPS_PER_REV_LOCAL);
  long targetKnee   = (long)((kneeDeg   / 360.0f) * EFFECTIVE_STEPS_PER_REV_LOCAL);

  motorHipYaw.moveTo(targetHipYaw);
  motorKnee.moveTo(targetKnee);

  // 4) Modo tractor: velocidad constante hacia el objetivo (sin rampas)
  motorHipYaw.setSpeed(targetHipYaw > motorHipYaw.currentPosition() ?  velocidad_constante : -velocidad_constante);
  motorKnee.setSpeed(  targetKnee   > motorKnee.currentPosition()   ?  velocidad_constante : -velocidad_constante);
}

void setup() {
  // micro-ROS por USB
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // UART al Mega (solo TX físico)
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // Ajustes de pulso (torque y limpieza)
  motorHipYaw.setMinPulseWidth(50);
  motorKnee.setMinPulseWidth(50);

  // Límites de seguridad (si vas en runSpeedToPosition, maxSpeed importa menos, pero déjalo razonable)
  motorHipYaw.setMaxSpeed(2000);
  motorKnee.setMaxSpeed(2000);

  delay(2000);

  // micro-ROS init
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
  rclc_executor_add_subscription(
    &executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA
  );
}

void loop() {
  // 1) Escuchar a ROS2/MoveIt
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));

  // 2) Mover motores locales X y Z a velocidad constante hasta el objetivo
  motorHipYaw.runSpeedToPosition();
  motorKnee.runSpeedToPosition();
}
