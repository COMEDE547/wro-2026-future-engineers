/*
 * motor_control.h — drive-motor control for the WRO FE vehicle (ESP32)
 * ====================================================================
 * Fills the "// Other robot tasks can go here (motor control, etc.)" gap in
 * the main firmware. Header-only: drop this file next to the .ino and
 * #include "motor_control.h".
 *
 * Supports the three driver topologies the car might end up with — pick one
 * with DRIVE_MODE below, wire per the matching table, done:
 *
 *   MODE_PWM_DIR  — EN/PWM + 2 direction pins.
 *                   L298N:   ENA->PIN_DRIVE_PWM, IN1/IN2->PIN_DRIVE_IN1/IN2
 *                   TB6612:  PWMA->PIN_DRIVE_PWM, AIN1/AIN2->IN1/IN2,
 *                            STBY->PIN_DRIVE_STBY (or tie STBY to 3V3)
 *   MODE_TWO_PIN  — PWM on both inputs, no separate enable.
 *                   DRV8871: IN1->PIN_DRIVE_PWM, IN2->PIN_DRIVE_IN2
 *                   BTS7960/IBT-2: RPWM->PIN_DRIVE_PWM, LPWM->PIN_DRIVE_IN2,
 *                            R_EN+L_EN tied HIGH
 *   MODE_ESC      — hobby car ESC on a servo-style signal (1000-2000 us).
 *                   Signal->PIN_DRIVE_PWM. NOTE: many car ESCs need a
 *                   brake/double-tap before reverse engages — test yours.
 *
 * Safety built in: bring-up speed cap (DRIVE_MAX), slew-rate ramp (no wheel-
 * spin / brownout on step commands), command watchdog (coast to stop if no
 * motorDrive() call for DRIVE_TIMEOUT_MS), instant stop that bypasses the
 * ramp. Works on arduino-esp32 core 2.x and 3.x (LEDC API changed in 3.0).
 *
 * API:
 *   motorBegin();                 // in setup(), after Serial.begin
 *   motorDrive(float s);          // every loop: s in [-1..1], +fwd -rev
 *   motorUpdate();                // every loop: applies ramp + watchdog
 *   motorStop(bool brake=false);  // instant stop; true = active brake
 *   motorCurrent();               // ramped speed actually applied
 *
 * Pin choice notes (ESP32): avoid 21/22 (I2C bus), 13 (steering servo),
 * strap pins 0/2/12/15, input-only 34-39, flash 6-11. Defaults below are
 * safe alongside the existing wiring.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// ======================================================================
// ==================  MOTOR CONFIG — EDIT THIS BLOCK  ==================
// ======================================================================

#define MODE_PWM_DIR 1
#define MODE_TWO_PIN 2
#define MODE_ESC     3

#ifndef DRIVE_MODE
#define DRIVE_MODE MODE_PWM_DIR      // <-- pick MODE_PWM_DIR / MODE_TWO_PIN / MODE_ESC
#endif

const int PIN_DRIVE_PWM  = 25;   // PWM_DIR: EN/PWM · TWO_PIN: input A · ESC: signal
const int PIN_DRIVE_IN1  = 26;   // PWM_DIR: direction 1 · others: unused
const int PIN_DRIVE_IN2  = 27;   // PWM_DIR: direction 2 · TWO_PIN: input B
const int PIN_DRIVE_STBY = -1;   // TB6612 STBY pin, -1 if tied high / not present

const bool  DRIVE_INVERT     = false;   // flip if "forward" runs backward
const float DRIVE_MAX        = 0.60f;   // bring-up cap — raise to 1.0 once
                                        // direction + steering trim verified
const float DRIVE_SLEW       = 3.0f;    // ramp: full-scale units per second
                                        // (0 -> 100% in ~330 ms). 0 = no ramp
const int   DRIVE_PWM_HZ     = 20000;   // 20 kHz = silent. L298N torque weak?
                                        // drop to 2000 (its switching is slow)
const int   DRIVE_TIMEOUT_MS = 300;     // watchdog: coast if no command. 0 = off

// ESC mode only:
const int ESC_MIN_US = 1000;
const int ESC_NEU_US = 1500;
const int ESC_MAX_US = 2000;
const int ESC_ARM_MS = 2000;            // neutral hold at boot so the ESC arms

// ======================================================================
// ===============  implementation — no edits needed below ==============
// ======================================================================

#if DRIVE_MODE == MODE_ESC
#include <ESP32Servo.h>
static Servo _escOut;
#endif

static const int   _PWM_RES_BITS = 10;
static const int   _PWM_MAX      = (1 << _PWM_RES_BITS) - 1;   // 1023
// High LEDC channels: ESP32Servo allocates upward from 0 for the steering
// servo — 14/15 keep the motor clear of it on core 2.x.
static const int   _LEDC_CH_A    = 14;
static const int   _LEDC_CH_B    = 15;

static float         _target    = 0.0f;   // commanded speed
static float         _current   = 0.0f;   // ramped speed actually applied
static unsigned long _lastCmdMs = 0;
static unsigned long _lastUpMs  = 0;
static bool          _braking   = false;

// ---- LEDC compatibility: core 3.x attaches by pin, 2.x by channel ----
static inline void _pwmAttach(int pin, int ch) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcAttach(pin, DRIVE_PWM_HZ, _PWM_RES_BITS);
#else
  ledcSetup(ch, DRIVE_PWM_HZ, _PWM_RES_BITS);
  ledcAttachPin(pin, ch);
#endif
}

static inline void _pwmWrite(int pin, int ch, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

// ---- apply a signed ramped speed to the hardware ----
static inline void _applyOutput(float s) {
  bool fwd = (s >= 0.0f) != DRIVE_INVERT;
  int  duty = (int)(fabsf(s) * _PWM_MAX + 0.5f);
  if (duty > _PWM_MAX) duty = _PWM_MAX;

#if DRIVE_MODE == MODE_PWM_DIR
  if (_braking) {
    digitalWrite(PIN_DRIVE_IN1, HIGH);
    digitalWrite(PIN_DRIVE_IN2, HIGH);
    _pwmWrite(PIN_DRIVE_PWM, _LEDC_CH_A, _PWM_MAX);
  } else {
    digitalWrite(PIN_DRIVE_IN1, fwd ? HIGH : LOW);
    digitalWrite(PIN_DRIVE_IN2, fwd ? LOW : HIGH);
    _pwmWrite(PIN_DRIVE_PWM, _LEDC_CH_A, duty);
  }

#elif DRIVE_MODE == MODE_TWO_PIN
  if (_braking) {                       // both high = brake on DRV8871/BTS7960
    _pwmWrite(PIN_DRIVE_PWM, _LEDC_CH_A, _PWM_MAX);
    _pwmWrite(PIN_DRIVE_IN2, _LEDC_CH_B, _PWM_MAX);
  } else if (fwd) {
    _pwmWrite(PIN_DRIVE_PWM, _LEDC_CH_A, duty);
    _pwmWrite(PIN_DRIVE_IN2, _LEDC_CH_B, 0);
  } else {
    _pwmWrite(PIN_DRIVE_PWM, _LEDC_CH_A, 0);
    _pwmWrite(PIN_DRIVE_IN2, _LEDC_CH_B, duty);
  }

#elif DRIVE_MODE == MODE_ESC
  (void)duty;
  float sv = fwd ? fabsf(s) : -fabsf(s);
  int us = ESC_NEU_US + (int)(sv * (sv >= 0 ? (ESC_MAX_US - ESC_NEU_US)
                                            : (ESC_NEU_US - ESC_MIN_US)));
  _escOut.writeMicroseconds(us);
#endif
}

// ---- public API ----
static inline void motorBegin() {
#if DRIVE_MODE == MODE_PWM_DIR
  pinMode(PIN_DRIVE_IN1, OUTPUT);
  pinMode(PIN_DRIVE_IN2, OUTPUT);
  digitalWrite(PIN_DRIVE_IN1, LOW);
  digitalWrite(PIN_DRIVE_IN2, LOW);
  if (PIN_DRIVE_STBY >= 0) {
    pinMode(PIN_DRIVE_STBY, OUTPUT);
    digitalWrite(PIN_DRIVE_STBY, HIGH);
  }
  _pwmAttach(PIN_DRIVE_PWM, _LEDC_CH_A);
  _pwmWrite(PIN_DRIVE_PWM, _LEDC_CH_A, 0);

#elif DRIVE_MODE == MODE_TWO_PIN
  _pwmAttach(PIN_DRIVE_PWM, _LEDC_CH_A);
  _pwmAttach(PIN_DRIVE_IN2, _LEDC_CH_B);
  _pwmWrite(PIN_DRIVE_PWM, _LEDC_CH_A, 0);
  _pwmWrite(PIN_DRIVE_IN2, _LEDC_CH_B, 0);

#elif DRIVE_MODE == MODE_ESC
  _escOut.setPeriodHertz(50);
  _escOut.attach(PIN_DRIVE_PWM, ESC_MIN_US, ESC_MAX_US);
  _escOut.writeMicroseconds(ESC_NEU_US);        // arm at neutral
  delay(ESC_ARM_MS);
#endif

  _target = _current = 0.0f;
  _braking = false;
  _lastCmdMs = _lastUpMs = millis();
}

// Command a speed in [-1..1]; call every loop iteration.
static inline void motorDrive(float s) {
  if (s >  DRIVE_MAX) s =  DRIVE_MAX;
  if (s < -DRIVE_MAX) s = -DRIVE_MAX;
  _target = s;
  _braking = false;
  _lastCmdMs = millis();
}

// Instant stop, bypassing the ramp. brake=true = active brake, else coast.
static inline void motorStop(bool brake = false) {
  _target = _current = 0.0f;
  _braking = brake;
  _lastCmdMs = millis();
  _applyOutput(0.0f);
}

// Call every loop: applies slew-rate ramp + command watchdog.
static inline void motorUpdate() {
  unsigned long now = millis();
  float dt = (now - _lastUpMs) * 0.001f;
  _lastUpMs = now;

  if (DRIVE_TIMEOUT_MS > 0 && (now - _lastCmdMs) > (unsigned long)DRIVE_TIMEOUT_MS
      && (_target != 0.0f || _current != 0.0f)) {
    motorStop(false);                    // watchdog: coast, don't slam brakes
    return;
  }

  if (DRIVE_SLEW > 0.0f) {
    float maxStep = DRIVE_SLEW * dt;
    float d = _target - _current;
    if (d >  maxStep) d =  maxStep;
    if (d < -maxStep) d = -maxStep;
    _current += d;
  } else {
    _current = _target;
  }
  _applyOutput(_current);
}

// Ramped speed actually being applied right now.
static inline float motorCurrent() { return _current; }

#endif // MOTOR_CONTROL_H
