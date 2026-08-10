/*
 * ESP32 + PCA9548A mux + 3x TF-Luna + BNO055 + steering servo
 * Drives a heading; when a side opens (corner), turns 90 deg and holds.
 * Libraries: Adafruit BNO055, Adafruit Unified Sensor, Adafruit BusIO, ESP32Servo
 * Wiring:
 *   PCA9548A SDA->GPIO21 SCL->GPIO22 (0x70)   TF-Luna ch0/1/2   BNO055 ch4 (0x28)
 *   Servo signal->GPIO13, V+->ext 5V, common GND
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP32Servo.h>

// ---------- Pins / addresses ----------
#define I2C_SDA       21
#define I2C_SCL       22
#define SERVO_PIN     13
#define PCA9548A_ADDR 0x70
#define TFLUNA_ADDR   0x10
#define BNO055_ADDR   0x28

// Mux channels
#define CH_LEFT       0
#define CH_CENTER     1
#define CH_RIGHT      2
#define CH_IMU        4

// ---------- Steering / turn tunables ----------
#define SERVO_MIN_ANGLE   64     // servo lower limit (deg)
#define SERVO_MAX_ANGLE   136    // servo upper limit (deg)
#define SERVO_CENTER      106    // wheels-straight angle
#define STEER_GAIN        1      // servo deg per deg of heading error
#define OPENING_CM        150    // side reading above this = corner/opening
#define TURN_STEP_DEG     90     // heading change per detected opening
#define STEER_DEADBAND    2.0
#define SERVO_PULSE_MIN   500    // us
#define SERVO_PULSE_MAX   2400   // us
#define LOOP_DELAY_MS     20

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDR, &Wire);
Servo servo;
float targetHeading = 0;   // the heading we steer to hold

// ---------- Mux ----------
void pcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ---------- Helpers ----------
float wrap180(float a) {
  while (a >  180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

// ---------- TF-Luna distance (cm), -1 if no response ----------
int readDistance(uint8_t channel) {
  pcaSelect(channel);
  delayMicroseconds(50);
  Wire.beginTransmission(TFLUNA_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom((uint8_t)TFLUNA_ADDR, (uint8_t)2) != 2) return -1;
  uint8_t lo = Wire.read();
  uint8_t hi = Wire.read();
  return lo | (hi << 8);
}

int left()   { return readDistance(CH_LEFT);   }
int center() { return readDistance(CH_CENTER); }
int right()  { return readDistance(CH_RIGHT);  }

// ---------- BNO055 ----------
float readHeading() {
  pcaSelect(CH_IMU);
  delayMicroseconds(50);
  imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return e.x();                     // yaw / heading, 0-360
}

void captureReference() {
  targetHeading = readHeading();    // current direction = heading to hold
}

// ---------- Steering: proportional correction toward targetHeading ----------
int steerToHeading(float h, float target) {
  float error = wrap180(h - target);
  if (fabsf(error) < STEER_DEADBAND) error = 0;
  float angle = SERVO_CENTER - STEER_GAIN * error;
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  Serial.printf("  [steer] err=%.1f angle=%.1f\n", error, angle);
  servo.write((int)angle);
  return (int)angle;
}

// ---------- Init ----------
void initIMU() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  pcaSelect(CH_IMU);
  delayMicroseconds(50);
  if (!bno.begin()) {               // OPERATION_MODE_IMUPLUS for motor-heavy robots
    Serial.println("BNO055 not found! Check ch4 wiring / address.");
    while (1) delay(10);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

void initServo() {
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
}

// ---------- Main ----------
void setup() {
  Serial.begin(115200);
  delay(300);
  initServo();
  delay(300);
  initIMU();
  initMotor();
  captureReference();
  Serial.println("\nReady: driving straight, turning at openings.\n");
  // pinMode(32, INPUT_PULLUP);int btn = digitalRead (32);
  // while(btn == 1){digitalRead (32);}
  startDrive();
}

void loop() {
  static unsigned long lidarResumeAt = 0;      // lunas paused until this millis()
  static bool leftWasOpen  = false;
  static bool rightWasOpen = false;

  float h = readHeading();                      // always read heading, always steer

  int l = -1, c = -1, r = -1;
  bool lidarPaused = (millis() < lidarResumeAt);

  if (!lidarPaused) {
    l = left();
    c = center();
    r = right();

    bool lOpen = (l > OPENING_CM);
    bool rOpen = (r > OPENING_CM);

    bool turned = false;
    if (lOpen && !leftWasOpen)  { targetHeading -= TURN_STEP_DEG; turned = true; }
    if (rOpen && !rightWasOpen) { targetHeading += TURN_STEP_DEG; turned = true; }

    leftWasOpen  = lOpen;
    rightWasOpen = rOpen;

    if (turned) lidarResumeAt = millis() + 1000;   // corner taken -> mute lunas 1 s
  }
  // during the pause: no reads, no corner detection — but steering stays live

  int servoAngle = steerToHeading(h, targetHeading);

  Serial.printf("L=%4d C=%4d R=%4d cm  H=%6.1f  Tgt=%6.1f  Servo=%3d  %s\n",
                l, c, r, h, targetHeading, servoAngle,
                lidarPaused ? "[lidar muted]" : "");

  trackTurnsAndStop();
  delay(LOOP_DELAY_MS);
}

// ============================================================================
// APPENDED — N20 drive motor via TB6612 (channel A). Plain motor module only.
// The logic above is untouched and behaves exactly as before.
//
// Wired in: initMotor() + startDrive() in setup(); trackTurnsAndStop() as
// one line at the end of loop(). Your corner/steering logic is unmodified —
// the counter only OBSERVES targetHeading steps your own logic makes.
//
// Wiring: VM->motor batt(+), VCC->3V3, GND common,
//         AIN1->GPIO25  AIN2->GPIO26  PWMA->GPIO33  STBY->GPIO27 (or tie to 3V3)
//         AO1/AO2 -> N20
// ============================================================================

#define MOTOR_AIN1         25
#define MOTOR_AIN2         26
#define MOTOR_PWMA         33
#define MOTOR_STBY         27      // set to -1 if STBY is hard-wired to 3V3
#define MOTOR_PWM_FREQ     20000   // Hz — above audible whine; TB6612 fine to 100 kHz
#define MOTOR_PWM_BITS     10      // duty range 0..1023
#define MOTOR_PWM_MAX      ((1 << MOTOR_PWM_BITS) - 1)
#define DRIVE_SPEED        1000    // cruise duty 0..1023 — tune on the mat
#define MOTOR_INVERT       false   // flip if the robot drives backward
#define START_DELAY_MS     1500    // hands-off pause inside startDrive()
#define MAX_TURNS          12      // 3 laps x 4 corners
#define FINAL_RUN_MS       500     // <-- THE knob: drive this long AFTER the last turn completes (~150 cm) - tune on the mat
#define TURN_SETTLED_DEG   15      // heading within this many deg of target = last turn considered complete
#define SETTLE_TIMEOUT_MS  4000    // failsafe: start the run-on anyway if heading never settles

ESP32PWM motorPwm;   // ESP32Servo's PWM class — shares LEDC timers with the Servo automatically

// TB6612 truth table: AIN1=H,AIN2=L -> fwd | L,H -> rev | H,H -> short brake | L,L + PWM=H -> coast
void motorBrake() {
  digitalWrite(MOTOR_AIN1, HIGH);
  digitalWrite(MOTOR_AIN2, HIGH);
  motorPwm.write(0);
}

void motorCoast() {
  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, LOW);
  motorPwm.write(MOTOR_PWM_MAX);   // L/L with PWM high = outputs high-Z
}

void setMotor(int speed) {   // -1023..1023, sign = direction, 0 = brake
  if (MOTOR_INVERT) speed = -speed;
  speed = constrain(speed, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  if (speed == 0) { motorBrake(); return; }
  digitalWrite(MOTOR_AIN1, speed > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_AIN2, speed > 0 ? LOW  : HIGH);
  motorPwm.write(abs(speed));
}

void initMotor() {
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  if (MOTOR_STBY >= 0) {
    pinMode(MOTOR_STBY, OUTPUT);
    digitalWrite(MOTOR_STBY, HIGH);   // TB6612 out of standby
  }
  motorPwm.attachPin(MOTOR_PWMA, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  motorBrake();                       // known-safe state before driving
}

void startDrive() {
  delay(START_DELAY_MS);              // hands-off pause
  setMotor(DRIVE_SPEED);
}

void trackTurnsAndStop() {
  static float prevTarget = NAN;               // first call: sync to current target
  static int turnCount = 0;
  static unsigned long finishAt = 0;
  static unsigned long lastTurnAt = 0;

  if (isnan(prevTarget)) prevTarget = targetHeading;

  if (targetHeading != prevTarget) {           // your loop() changed the target -> corner(s) taken
    turnCount += (int)(fabsf(targetHeading - prevTarget) / TURN_STEP_DEG + 0.5f);
    prevTarget = targetHeading;
    Serial.printf("[turn] %d/%d\n", turnCount, MAX_TURNS);
  }

  if (turnCount >= MAX_TURNS && finishAt == 0) {
    if (lastTurnAt == 0) lastTurnAt = millis();
    float err = fabsf(wrap180(readHeading() - targetHeading));   // extra IMU read, this phase only
    if (err < TURN_SETTLED_DEG || millis() - lastTurnAt > SETTLE_TIMEOUT_MS) {
      finishAt = millis() + FINAL_RUN_MS;                        // last turn done -> timed run-on starts NOW
      Serial.println("[turn] last turn settled - final run-on");
    }
  }
  if (finishAt != 0 && millis() >= finishAt) { // steering stayed live through the run-on
    motorBrake();
    Serial.println("Run complete - motor braked.");
    while (1) delay(100);
  }
}
