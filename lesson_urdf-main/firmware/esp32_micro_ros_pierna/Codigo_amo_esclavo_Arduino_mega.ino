#include <Arduino.h>
#include <AccelStepper.h>

// =========================
// PINES (solo estos)
// =========================
#define C_STEP  2
#define C_DIR   3

// =========================
// CINEMÁTICA CONSISTENTE CON EL MAESTRO
// =========================
static constexpr float STEPS_PER_REV = 200.0f;
static constexpr float GEAR_RATIO    = 11.0f;
static constexpr float MICROSTEPS    = 8.0f;
static constexpr float EFFECTIVE_STEPS_PER_REV = STEPS_PER_REV * MICROSTEPS * GEAR_RATIO;

// =========================
// MOTOR (driver tipo STEP/DIR: M542T u otro)
// =========================
AccelStepper motorC(AccelStepper::DRIVER, C_STEP, C_DIR);

// =========================
// PARÁMETROS "ALTO TORQUE"
// Clave: baja velocidad + pulso limpio
// =========================
static constexpr uint16_t MIN_PULSE_US = 50;     // como tu ejemplo de torque
static constexpr float MAX_SPEED_STEPS = 2000.0f; // pasos/seg (bajo = más torque)
static constexpr float ACCEL_STEPS     = 1000.0;  // suave para no perder pasos

// =========================
// UART parser (sin String)
// =========================
static char rxbuf[32];
static uint8_t rxlen = 0;

static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Parser rápido para float (usa atof, suficiente y estable)
static void handleLine(const char* s) {
  // Espera: "C:-12.34"
  if (s[0] != 'C' || s[1] != ':') return;

  float deg = atof(s + 2);
  deg = clampf(deg, -90.0f, 90.0f);

  // deg -> pasos
  const long target = (long)((deg / 360.0f) * EFFECTIVE_STEPS_PER_REV);
  motorC.moveTo(target);

  // (opcional) ACK
  // Serial1.print("ACK\n");
}

void setup() {
  Serial.begin(115200);   // debug USB
  Serial1.begin(115200);  // desde ESP32 (TX->RX1=19)

  motorC.setMinPulseWidth(MIN_PULSE_US);

  // Alto torque: conservador
  motorC.setMaxSpeed(MAX_SPEED_STEPS);
  motorC.setAcceleration(ACCEL_STEPS);

  // Dirección inicial (opcional)
  // digitalWrite(C_DIR, LOW);

  Serial.println("MEGA SLAVE listo. Recibo: C:<deg>  | STEP=2 DIR=3 | sin EN | alto torque");
}

void loop() {
  // ---- RX UART (no bloqueante) ----
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    Serial.write(c); // muestra en monitor USB lo que entra por pin 19


    if (c == '\n') {
      rxbuf[rxlen] = '\0';
      handleLine(rxbuf);
      rxlen = 0;
    } else if (c != '\r') {
      if (rxlen < (sizeof(rxbuf) - 1)) {
        rxbuf[rxlen++] = c;
      } else {
        // overflow -> reset buffer
        rxlen = 0;
      }
    }
  }

  // ---- Movimiento ----
  motorC.run();
}
