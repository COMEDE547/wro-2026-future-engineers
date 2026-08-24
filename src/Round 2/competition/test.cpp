// Drive firmware — ESP32_Robot.ino
// USB serial to Pi: handshake starts when pin 32 is pressed
// (ESP_READY / PI_HELLO / ESP_ACK), then DRIVE / RED / GREEN / REVERSE / CLEAR / TRACKING.
// No WiFi tuner. Telemetry is prefixed TEL so the Pi ignores it.
//
// Corner: front TF-Luna FRONT_TURN_DISTANCE (20 cm) → reverse-arc 90°.
// If the Pi is tracking a cube when that lidar fires:
//   close cube (height >= BLOCK_IN_FRONT_PX) → IMU reverse on cardinal, then pass
//   far cube still in view → IMU reverse on cardinal, then the 90° turn
// Obstacle pass: 400ms PAUSE at 45px (motors 0, pass steer held), then wide
// PASS until the cube leaves the camera, SIDE (straight) until the matching
// side LiDAR (red=left, green=right), then yaw back
// (YAW_BACK_DEG / YAW_BACK_SPEED / YAW_BACK_MS). Red = RIGHT of cube.
// Boot: wait for pin-32 button, then S-curve park exit (reverse, 45° away,
// straight, 45° back) using L/R lidars for slot side. Then lane center.

#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>

#define SERVO_PIN 13
#define MOTOR_IN1  25
#define MOTOR_IN2  26
#define MOTOR_PWM  33
#define MUX_CH_LEFT   0
#define MUX_CH_CENTER 1
#define MUX_CH_RIGHT  2
#define MUX_CH_BNO    4
#define TFLUNA_I2C_ADDR 0x10
#define PARK_BUTTON_PIN 32
#define READY_LED_PIN   2

int STRAIGHT_SPEED = 220;     // open lane
int TURN_SPEED     = 200;
int BACKWARD_SPEED = -180;    // IMU reverse (cube too close / stuck pass / pre-turn)
int AVOID_SPEED    = 150;     // fallback if a DRIVE line has no speed
int AVOID_SPEED_MIN = 60;
int AVOID_SPEED_MAX = 200;    // pass never reaches STRAIGHT_SPEED
int AVOID_STUCK_SPEED = 110;  // after PASS_STUCK_MS, still slower + full steer
int REJOIN_SPEED   = 140;
int PARK_SPEED     = 150;     // S-curve exit (fwd and reverse use ± this)

const int RAMP_START_SPEED = 80;
const int RAMP_STEP = 30;
const unsigned long RAMP_DURATION_MS = 2000;

int SERVO_CENTER   = 75;
int DIFF = 35;
int SERVO_MAX_LEFT = SERVO_CENTER + DIFF;
int SERVO_MAX_RIGHT= SERVO_CENTER - DIFF;

const bool INVERT_STEERING = true;
int steerTowardHeading(float targetHeading, float currentHeading);
void resumeStraightDriving();
void resetPiSerialRx();
void trackTurnsAndStop();
int servoWithOffset(bool steerRight, int offsetDeg);
bool lineChecksumOk(const String& line);
void beginSideWait();
void beginYawBack();
void executeSideWait(float currentHeading);
void executeYawBack(float currentHeading);
bool inSideOrYaw();

float STEERING_KP  = 1.5;
float STEERING_KI  = 0.02;
float STEERING_KD  = 0.15;
const float HEADING_DEADBAND = 1.5;
const int   MAX_STEER_CORRECTION = 20;
const int MAX_TURNS      = 12;   // 3 laps × 4 corners
const int MAX_BLOCKS     = 8;    // WRO obstacle slots on the mat

unsigned long turnCooldownUntil = 0;
bool avoidDirectionRight = true;
int avoidServoOffset = 18;
int avoidDriveSpeed = AVOID_SPEED;
unsigned long lastObstacleCmd = 0;
unsigned long avoidPassStartMs = 0;
bool stuckSteerUsed = false;
bool stuckReverseUsed = false;
bool holdPathActive = false;  // unused HOLD path — Pi now uses SIDE instead
bool pauseAlignActive = false;  // Pi PAUSE: motors 0 for 400 ms, servo held at pass angle
int pauseServoHeld = -1;        // freeze pause steer so the servo does not twitch
int lastWrittenServo = -999;
const unsigned long CUBE_RETRY_REVERSE_MS = 550;   // short reverse, then try the pass again
const unsigned long PASS_STUCK_MS = 2500;          // still seeing cube → full DIFF
const unsigned long PASS_STUCK_REVERSE_MS = 4200;  // still not passed → IMU reverse, retry
const unsigned long OBSTACLE_TIMEOUT_MS = 1200;

// After the cube is off-camera: drive straight, then yaw back to the lane.
// Tune these three for the yaw: degrees of servo, motor PWM, duration.
int SIDE_CUBE_CM = 20;                 // matching side LiDAR (red=L, green=R)
unsigned long SIDE_MIN_MS = 80;        // ignore lidar for this long after SIDE starts
unsigned long SIDE_CONFIRM_MS = 40;    // debounce
unsigned long SIDE_WAIT_MAX_MS = 1600; // yaw anyway if the cube never trips lidar
int YAW_BACK_DEG = 28;                 // servo offset toward the lane
int YAW_BACK_SPEED = 140;              // PWM during the yaw
unsigned long YAW_BACK_MS = 500;       // how long to hold that yaw
float sideHoldHeading = 0.0;
unsigned long sideWaitStartMs = 0;
unsigned long sideConfirmStartMs = 0;
unsigned long yawBackStartMs = 0;
const unsigned long TURN_COOLDOWN_MS = 1000;
const unsigned long AVOID_LIDAR_IGNORE_MS = 2500;

const float CARDINAL_HEADINGS[4] = {0.0, 90.0, 180.0, 270.0};

int FRONT_TURN_DISTANCE = 20;
bool frontConditionActive = false;
unsigned long frontConditionStartTime = 0;
const unsigned long FRONT_CONFIRM_MS = 150;

int ARC_SERVO_ANGLE = 35;
float ARC_EXIT_THRESHOLD = 2.0;
unsigned long ARC_PAUSE_MS = 500;
unsigned long ARC_MIN_MS = 800;
unsigned long ARC_MAX_MS = 4000;

enum TurnPhase { PHASE_PAUSE, PHASE_REVERSE };
TurnPhase currentTurnPhase = PHASE_PAUSE;
unsigned long turnPhaseStartTime = 0;
unsigned long arcStartTime = 0;

const float RECENTER_HEADING_DEG = 12.0;
int SIDE_AVOID_CM = 12;
int SIDE_AVOID_SERVO = 28;
int PASS_WALL_CM = 8;           // only ease pass steer if the OUTER wall is this close
int REJOIN_BALANCE_CM = 8;
unsigned long REJOIN_HOLD_MS = 150;
unsigned long REJOIN_MAX_MS = 2000;
unsigned long PRE_TURN_REVERSE_MS = 650;
int PRE_TURN_CLEAR_CM = 28;
int BLOCK_IN_FRONT_PX = 40;     // cube this tall + front lidar = cube, not corner wall
unsigned long TRACKING_HOLD_MS = 600;

// Parking S-curve (same sequence as the parking-exit test sketch).
int PARK_EXIT_ANGLE_DEG = 45;
float PARK_EXIT_THRESHOLD = 8.0;
unsigned long PARK_SAFETY_CAP_MS = 8000;
unsigned long PARK_STRAIGHT_MS = 800;   // forward burst out of the stall — raise to come out more
unsigned long PARK_REVERSE_MS = 650;    // first reverse out of the slot
int HEADING_DIR = -1;                 // BNO: right turn increases heading
float PARK_ROTATION_MARGIN_DEG = 30.0;
bool parkDirectionRight = true;
float parkStartHeading = 0.0;
float parkTargetHeading1 = 0.0;
float parkTargetHeading2 = 0.0;
float parkPhaseStartHeading = 0.0;
unsigned long parkPhaseStartTime = 0;
bool systemReady = false;
unsigned long parkButtonDownMs = 0;

const unsigned long TELEMETRY_MS = 80;
const unsigned long ESP_READY_PERIOD_MS = 400;

bool piLinkOk = false;
bool handshakeArmed = false;
unsigned long lastReadyAdvert = 0;
unsigned long lastPiCmd = 0;
unsigned long lastTelemetryMs = 0;
bool piTracking = false;
int trackedHeightPx = 0;
bool trackedPassRight = true;
unsigned long lastTrackingMs = 0;
bool reverseThenAvoid = false;
float reverseHoldHeading = 0.0;
unsigned long reverseStartMs = 0;
unsigned long rejoinStartMs = 0;
unsigned long rejoinBalancedStart = 0;
float pathHeadingBeforeBlock = 0.0;
bool pathHeadingCaptured = false;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo steeringServo;

enum RobotState {
  DRIVING_STRAIGHT,
  TURNING,
  ROBOT_STOPPED,
  OBSTACLE_AVOIDING,
  PI_REVERSE,
  PRE_TURN_REVERSE,
  PI_REJOIN,
  PI_SIDE,
  PI_YAW_BACK,
  PARK_WAIT,
  PARK_REVERSE,
  PARK_PHASE1,
  PARK_STRAIGHT_LEG,
  PARK_PHASE2
};
RobotState currentState = PARK_WAIT;

float straightTargetHeading = 0.0;
float turnTargetHeading     = 0.0;
bool isTurningLeft          = false;

int totalTurnsCount         = 0;
bool hasTurnedOnce          = false;
bool cubesEnabled           = false;  // after first reverse-arc corner, Pi DRIVE is allowed
bool lockedDirectionLeft    = false;

int16_t currentLeftDist     = -1;
int16_t currentCenterDist   = -1;
int16_t currentRightDist    = -1;
int finalServoAngle         = 90;
float headingError          = 0.0;
float angleDifference       = 0.0;
float lastHeadingError       = 0.0;
float integralError          = 0.0;
unsigned long lastHeadingTime = 0;

unsigned long driveStartTime = 0;
bool rampActive = false;
bool rampArmedForThisPhase = false;

void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

int16_t getLunaDistance(uint8_t channel) {
  selectMuxChannel(channel);
  Wire.beginTransmission(TFLUNA_I2C_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return -1;
  Wire.requestFrom(TFLUNA_I2C_ADDR, 2);
  if (Wire.available() >= 2) {
    uint8_t lowByte = Wire.read();
    uint8_t highByte = Wire.read();
    return (lowByte + (highByte << 8));
  }
  return -1;
}

float getCurrentHeading() {
  selectMuxChannel(MUX_CH_BNO);
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

float filteredHeading = 0.0;
bool headingInitialized = false;

bool stopConditionActive             = false;
unsigned long stopConditionStartTime = 0;
const unsigned long STOP_CONFIRM_MS  = 1000;

float getSmoothedHeading() {
  float raw = getCurrentHeading();
  if (!headingInitialized) {
    filteredHeading = raw;
    headingInitialized = true;
    return filteredHeading;
  }
  float diff = raw - filteredHeading;
  if (diff > 180.0)  diff -= 360.0;
  if (diff < -180.0) diff += 360.0;
  filteredHeading += 0.2 * diff;
  if (filteredHeading < 0.0)    filteredHeading += 360.0;
  if (filteredHeading >= 360.0) filteredHeading -= 360.0;
  return filteredHeading;
}

float snapToCardinal(float angle) {
  angle = fmod(angle, 360.0);
  if (angle < 0.0) angle += 360.0;
  float best = CARDINAL_HEADINGS[0];
  float bestDiff = 999.0;
  for (int i = 0; i < 4; i++) {
    float diff = fabs(angle - CARDINAL_HEADINGS[i]);
    if (diff > 180.0) diff = 360.0 - diff;
    if (diff < bestDiff) {
      bestDiff = diff;
      best = CARDINAL_HEADINGS[i];
    }
  }
  return best;
}

float computeTurnTarget(float currentHeading, bool turningLeft) {
  float raw = turningLeft ? (currentHeading - 90.0) : (currentHeading + 90.0);
  raw = fmod(raw, 360.0);
  if (raw < 0.0) raw += 360.0;
  return snapToCardinal(raw);
}

float shortestAngleDiff(float fromHeading, float toHeading) {
  float diff = fromHeading - toHeading;
  if (diff > 180.0)  diff -= 360.0;
  if (diff < -180.0) diff += 360.0;
  return diff;
}

float wrapHeading(float deg) {
  deg = fmod(deg, 360.0);
  if (deg < 0.0) deg += 360.0;
  return deg;
}

void writeServoAngle(int angle) {
  angle = constrain(angle, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
  if (abs(angle - lastWrittenServo) < 2) {
    return;
  }
  lastWrittenServo = angle;
  steeringServo.write(angle);
}

void setMotorOutput(int speed) {
  if (speed >= 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    speed = -speed;
  }
  analogWrite(MOTOR_PWM, constrain(speed, 0, 255));
}

int getRampedSpeed(int targetSpeed) {
  if (!rampActive) return targetSpeed;
  unsigned long elapsed = millis() - driveStartTime;
  if (elapsed >= RAMP_DURATION_MS) {
    rampActive = false;
    return targetSpeed;
  }
  int numSteps = max(1, (targetSpeed - RAMP_START_SPEED) / RAMP_STEP);
  unsigned long stepDuration = RAMP_DURATION_MS / numSteps;
  int stepIndex = elapsed / stepDuration;
  int speed = RAMP_START_SPEED + stepIndex * RAMP_STEP;
  return constrain(speed, RAMP_START_SPEED, targetSpeed);
}

void ignoreFrontLidarFor(unsigned long ms) {
  unsigned long until = millis() + ms;
  if (until > turnCooldownUntil) turnCooldownUntil = until;
  frontConditionActive = false;
}

bool inParkManeuver() {
  return currentState == PARK_WAIT || currentState == PARK_REVERSE ||
         currentState == PARK_PHASE1 || currentState == PARK_STRAIGHT_LEG ||
         currentState == PARK_PHASE2;
}

bool inSideOrYaw() {
  return currentState == PI_SIDE || currentState == PI_YAW_BACK;
}

int parkCrankServoAngle(bool crankRight) {
  return crankRight ? SERVO_MAX_RIGHT : SERVO_MAX_LEFT;
}

void beginLaneAfterParking() {
  ignoreFrontLidarFor(AVOID_LIDAR_IGNORE_MS);
  rejoinStartMs = millis();
  rejoinBalancedStart = 0;
  pathHeadingCaptured = true;
  straightTargetHeading = snapToCardinal(parkStartHeading);
  pathHeadingBeforeBlock = straightTargetHeading;
  currentState = PI_REJOIN;
  Serial.println("TEL PARK S-curve done — lane center");
}

void startParkSCurve() {
  parkDirectionRight = (currentRightDist > currentLeftDist);
  parkStartHeading = getSmoothedHeading();
  parkTargetHeading1 = wrapHeading(parkStartHeading +
      HEADING_DIR * (parkDirectionRight ? -PARK_EXIT_ANGLE_DEG : PARK_EXIT_ANGLE_DEG));
  parkTargetHeading2 = parkStartHeading;
  parkPhaseStartTime = millis();
  parkPhaseStartHeading = parkStartHeading;
  currentState = PARK_REVERSE;
  handshakeArmed = true;
  lastReadyAdvert = 0;
  Serial.println("ESP_READY");
  Serial.print("TEL PARK EXIT slot side=");
  Serial.println(parkDirectionRight ? "RIGHT" : "LEFT");
  Serial.println("TEL handshake armed — waiting for PI_HELLO");
}

void checkParkButton() {
  if (!systemReady || currentState != PARK_WAIT) return;
  if (digitalRead(PARK_BUTTON_PIN) == LOW) {
    if (parkButtonDownMs == 0) parkButtonDownMs = millis();
    if (millis() - parkButtonDownMs < 40) return;
    parkButtonDownMs = 0;
    startParkSCurve();
  } else {
    parkButtonDownMs = 0;
  }
}

bool trackingFresh() {
  return piTracking && (millis() - lastTrackingMs <= TRACKING_HOLD_MS);
}

void capturePathHeadingBeforeBlock() {
  if (pathHeadingCaptured) return;
  pathHeadingCaptured = true;
  pathHeadingBeforeBlock = snapToCardinal(straightTargetHeading);
}

int servoWithOffset(bool steerRight, int offsetDeg) {
  int offset = constrain(offsetDeg, 0, DIFF);
  int angle;
  if (INVERT_STEERING) {
    angle = steerRight ? (SERVO_CENTER + offset) : (SERVO_CENTER - offset);
  } else {
    angle = steerRight ? (SERVO_CENTER - offset) : (SERVO_CENTER + offset);
  }
  return constrain(angle, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
}

int parsePiIntField(String line, int fieldIndex, int fallback) {
  int star = line.indexOf('*');
  if (star >= 0) line = line.substring(0, star);
  int start = 0;
  int idx = 0;
  while (idx < fieldIndex) {
    int comma = line.indexOf(',', start);
    if (comma < 0) return fallback;
    start = comma + 1;
    idx++;
  }
  int comma = line.indexOf(',', start);
  String field = (comma < 0) ? line.substring(start) : line.substring(start, comma);
  field.trim();
  if (field.length() == 0) return fallback;
  return field.toInt();
}

String parsePiField(String line, int fieldIndex) {
  int star = line.indexOf('*');
  if (star >= 0) line = line.substring(0, star);
  int start = 0;
  int idx = 0;
  while (idx < fieldIndex) {
    int comma = line.indexOf(',', start);
    if (comma < 0) return "";
    start = comma + 1;
    idx++;
  }
  int comma = line.indexOf(',', start);
  String field = (comma < 0) ? line.substring(start) : line.substring(start, comma);
  field.trim();
  return field;
}

bool lineChecksumOk(const String& line) {
  int star = line.lastIndexOf('*');
  if (star < 0) return true;
  if (star + 3 > (int)line.length()) return false;
  String hex = line.substring(star + 1);
  hex.trim();
  hex.toUpperCase();
  unsigned int expected = 0;
  for (unsigned int i = 0; i < hex.length(); i++) {
    char c = hex[i];
    expected <<= 4;
    if (c >= '0' && c <= '9') expected += (unsigned int)(c - '0');
    else if (c >= 'A' && c <= 'F') expected += (unsigned int)(c - 'A' + 10);
    else return false;
  }
  uint8_t got = 0;
  for (int i = 0; i < star; i++) {
    got ^= (uint8_t)line[i];
  }
  return got == (uint8_t)expected;
}

int steerTowardHeading(float targetHeading, float currentHeading) {
  float rawError = targetHeading - currentHeading;
  if (rawError > 180.0)  rawError -= 360.0;
  if (rawError < -180.0) rawError += 360.0;
  headingError = rawError;

  unsigned long now = millis();
  float dt = (now - lastHeadingTime) / 1000.0;
  if (dt < 0.001) dt = 0.001;
  float errorRate = (rawError - lastHeadingError) / dt;
  float pTermInput = (abs(rawError) < HEADING_DEADBAND) ? 0.0 : rawError;
  if (abs(rawError) >= HEADING_DEADBAND) {
    integralError += rawError * dt;
  }
  const float MAX_INTEGRAL = 50.0;
  integralError = constrain(integralError, -MAX_INTEGRAL, MAX_INTEGRAL);

  int steeringCorrection = (int)round(pTermInput * STEERING_KP + integralError * STEERING_KI + errorRate * STEERING_KD);
  steeringCorrection = constrain(steeringCorrection, -MAX_STEER_CORRECTION, MAX_STEER_CORRECTION);

  lastHeadingError = rawError;
  lastHeadingTime = now;

  int angle;
  if (INVERT_STEERING) {
    angle = SERVO_CENTER + steeringCorrection;
  } else {
    angle = SERVO_CENTER - steeringCorrection;
  }
  return constrain(angle, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
}

void beginCornerTurn() {
  float currentHeading = getSmoothedHeading();
  bool turnLeft;
  if (hasTurnedOnce) {
    turnLeft = lockedDirectionLeft;
  } else {
    turnLeft = (currentLeftDist > currentRightDist);
    hasTurnedOnce = true;
    lockedDirectionLeft = turnLeft;
    if (turnLeft) {
      Serial.println("TEL LAYOUT: PERMANENT LEFT TURN");
    } else {
      Serial.println("TEL LAYOUT: PERMANENT RIGHT TURN");
    }
  }
  isTurningLeft = turnLeft;
  turnTargetHeading = computeTurnTarget(currentHeading, turnLeft);
  currentTurnPhase = PHASE_PAUSE;
  turnPhaseStartTime = millis();
  arcStartTime = 0;
  currentState = TURNING;
  frontConditionActive = false;
  rampArmedForThisPhase = false;
  reverseThenAvoid = false;
}

void beginPreTurnReverse() {
  reverseHoldHeading = snapToCardinal(
      pathHeadingCaptured ? pathHeadingBeforeBlock : straightTargetHeading);
  reverseStartMs = millis();
  currentState = PRE_TURN_REVERSE;
  frontConditionActive = false;
  rampArmedForThisPhase = false;
  Serial.println("TEL PRE-TURN reverse on cardinal, then 90 deg arc");
}

void beginCubeReverseThenAvoid() {
  capturePathHeadingBeforeBlock();
  reverseHoldHeading = snapToCardinal(
      pathHeadingCaptured ? pathHeadingBeforeBlock : straightTargetHeading);
  reverseStartMs = millis();
  reverseThenAvoid = true;
  currentState = PI_REVERSE;
  frontConditionActive = false;
  ignoreFrontLidarFor(AVOID_LIDAR_IGNORE_MS);
  Serial.println("TEL cube in front — IMU reverse then grid pass");
}

void checkFrontObstacle() {
  if (millis() < turnCooldownUntil) return;
  if (currentCenterDist <= 0) { frontConditionActive = false; return; }

  if (currentCenterDist < FRONT_TURN_DISTANCE) {
    if (!frontConditionActive) {
      frontConditionActive = true;
      frontConditionStartTime = millis();
      return;
    }
    if (millis() - frontConditionStartTime < FRONT_CONFIRM_MS) return;

    if (trackingFresh() && trackedHeightPx >= BLOCK_IN_FRONT_PX) {
      beginCubeReverseThenAvoid();
      return;
    }
    if (trackingFresh() && trackedHeightPx > 0) {
      beginPreTurnReverse();
      return;
    }
    beginCornerTurn();
  } else {
    frontConditionActive = false;
  }
}

void driveStraightMode(float currentHeading) {
  if (!rampArmedForThisPhase) { driveStartTime = millis(); rampActive = true; rampArmedForThisPhase = true; }
  setMotorOutput(getRampedSpeed(STRAIGHT_SPEED));
  finalServoAngle = steerTowardHeading(straightTargetHeading, currentHeading);
  steeringServo.write(finalServoAngle);
}

int reversingArcServoAngle() {
  int offset = constrain(ARC_SERVO_ANGLE, 0, DIFF);
  int leftExtreme  = INVERT_STEERING ? (SERVO_CENTER - offset) : (SERVO_CENTER + offset);
  int rightExtreme = INVERT_STEERING ? (SERVO_CENTER + offset) : (SERVO_CENTER - offset);
  int reverseCranked = isTurningLeft ? rightExtreme : leftExtreme;
  return constrain(reverseCranked, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
}

void finishArcTurn() {
  totalTurnsCount++;
  if (!cubesEnabled) {
    cubesEnabled = true;
    Serial.println("TEL CUBES ON — first reverse-arc done, Pi may DRIVE");
  }
  Serial.print("TEL ARC DONE turns=");
  Serial.println(totalTurnsCount);
  setMotorOutput(0);
  steeringServo.write(SERVO_CENTER);
  finalServoAngle = SERVO_CENTER;
  straightTargetHeading = turnTargetHeading;
  pathHeadingCaptured = false;
  currentState = DRIVING_STRAIGHT;
  currentTurnPhase = PHASE_PAUSE;
  rampArmedForThisPhase = false;
  turnCooldownUntil = millis() + TURN_COOLDOWN_MS;
  integralError = 0.0;
  lastHeadingError = 0.0;
  lastHeadingTime = millis();
}

void executeTurnMode(float currentHeading) {
  unsigned long now = millis();
  angleDifference = shortestAngleDiff(currentHeading, turnTargetHeading);

  if (currentTurnPhase == PHASE_PAUSE) {
    setMotorOutput(0);
    steeringServo.write(SERVO_CENTER);
    finalServoAngle = SERVO_CENTER;
    if (now - turnPhaseStartTime >= ARC_PAUSE_MS) {
      currentTurnPhase = PHASE_REVERSE;
      arcStartTime = now;
    }
    return;
  }

  unsigned long elapsed = now - arcStartTime;
  int crankedAngle = reversingArcServoAngle();
  setMotorOutput(BACKWARD_SPEED);
  steeringServo.write(crankedAngle);
  finalServoAngle = crankedAngle;

  float remainingAngle = abs(angleDifference);
  bool minTimeReached = elapsed >= ARC_MIN_MS;
  bool maxTimeReached = elapsed >= ARC_MAX_MS;
  bool headingClose   = remainingAngle <= ARC_EXIT_THRESHOLD;

  if ((minTimeReached && headingClose) || maxTimeReached) {
    if (maxTimeReached && !headingClose) {
      Serial.println("TEL ARC: max time — exiting with heading still off");
    }
    finishArcTurn();
  }
}

void resumeStraightDriving() {
  currentState = DRIVING_STRAIGHT;
  reverseThenAvoid = false;
  pathHeadingCaptured = false;
  rampArmedForThisPhase = false;
  lastHeadingError = 0.0;
  lastHeadingTime = millis();
  integralError = 0.0;
}

int wallSafeServo(int requested) {
  bool leftClose  = (currentLeftDist  > 0 && currentLeftDist  < SIDE_AVOID_CM);
  bool rightClose = (currentRightDist > 0 && currentRightDist < SIDE_AVOID_CM);
  int away = constrain(SIDE_AVOID_SERVO, 0, DIFF);
  if (leftClose && rightClose) {
    return SERVO_CENTER;
  }
  int angle = requested;
  if (leftClose) {
    int avoidRight = INVERT_STEERING ? (SERVO_CENTER + away) : (SERVO_CENTER - away);
    angle = INVERT_STEERING ? max(angle, avoidRight) : min(angle, avoidRight);
  } else if (rightClose) {
    int avoidLeft = INVERT_STEERING ? (SERVO_CENTER - away) : (SERVO_CENTER + away);
    angle = INVERT_STEERING ? min(angle, avoidLeft) : max(angle, avoidLeft);
  }
  return constrain(angle, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
}

int passSafeServo(int requested) {
  // Extra steer away from the cube only. Never ease toward the cube
  // because the outer wall is close — that cancelled green (pass-left)
  // and drove the body into the block.
  int away = constrain(SIDE_AVOID_SERVO, 0, DIFF);
  int angle = requested;
  if (avoidDirectionRight) {
    bool cubeLeft = (currentLeftDist > 0 && currentLeftDist < SIDE_AVOID_CM);
    if (cubeLeft) {
      int moreRight = INVERT_STEERING ? (SERVO_CENTER + away) : (SERVO_CENTER - away);
      angle = INVERT_STEERING ? max(angle, moreRight) : min(angle, moreRight);
    }
    if (INVERT_STEERING) angle = max(angle, requested);
    else angle = min(angle, requested);
  } else {
    bool cubeRight = (currentRightDist > 0 && currentRightDist < SIDE_AVOID_CM);
    if (cubeRight) {
      int moreLeft = INVERT_STEERING ? (SERVO_CENTER - away) : (SERVO_CENTER + away);
      angle = INVERT_STEERING ? min(angle, moreLeft) : max(angle, moreLeft);
    }
    if (INVERT_STEERING) angle = min(angle, requested);
    else angle = max(angle, requested);
  }
  return constrain(angle, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
}

void beginSideWait() {
  currentState = PI_SIDE;
  sideWaitStartMs = millis();
  sideConfirmStartMs = 0;
  sideHoldHeading = getSmoothedHeading();
  lastObstacleCmd = millis();
  stuckSteerUsed = false;
  stuckReverseUsed = false;
  holdPathActive = false;
  ignoreFrontLidarFor(AVOID_LIDAR_IGNORE_MS);
  Serial.println("TEL SIDE wait — straight until matching LiDAR");
}

void beginYawBack() {
  currentState = PI_YAW_BACK;
  yawBackStartMs = millis();
  lastObstacleCmd = millis();
  stuckSteerUsed = false;
  stuckReverseUsed = false;
  ignoreFrontLidarFor(AVOID_LIDAR_IGNORE_MS);
  Serial.print("TEL YAW BACK deg=");
  Serial.print(YAW_BACK_DEG);
  Serial.print(" spd=");
  Serial.print(YAW_BACK_SPEED);
  Serial.print(" ms=");
  Serial.println(YAW_BACK_MS);
}

void executeSideWait(float currentHeading) {
  int speed = constrain(avoidDriveSpeed, AVOID_SPEED_MIN, AVOID_SPEED_MAX);
  setMotorOutput(speed);
  int angle = steerTowardHeading(sideHoldHeading, currentHeading);
  writeServoAngle(angle);
  finalServoAngle = angle;

  unsigned long now = millis();
  unsigned long elapsed = now - sideWaitStartMs;
  int sideCm = avoidDirectionRight ? currentLeftDist : currentRightDist;
  bool cubeOnPassSide = (sideCm > 0 && sideCm < SIDE_CUBE_CM);
  if (elapsed >= SIDE_MIN_MS && cubeOnPassSide) {
    if (sideConfirmStartMs == 0) sideConfirmStartMs = now;
    else if (now - sideConfirmStartMs >= SIDE_CONFIRM_MS) {
      beginYawBack();
      return;
    }
  } else {
    sideConfirmStartMs = 0;
  }
  if (elapsed >= SIDE_WAIT_MAX_MS) {
    Serial.println("TEL SIDE timeout — yaw back anyway");
    beginYawBack();
  }
}

void executeYawBack(float currentHeading) {
  (void)currentHeading;
  int speed = constrain(YAW_BACK_SPEED, AVOID_SPEED_MIN, AVOID_SPEED_MAX);
  setMotorOutput(speed);
  int offset = constrain(YAW_BACK_DEG, 0, DIFF);
  int angle = servoWithOffset(!avoidDirectionRight, offset);
  writeServoAngle(angle);
  finalServoAngle = angle;
  if (millis() - yawBackStartMs >= YAW_BACK_MS) {
    rejoinStartMs = millis();
    rejoinBalancedStart = 0;
    currentState = PI_REJOIN;
    Serial.println("TEL YAW DONE");
  }
}

void avoidObstacle(float currentHeading) {
  if (pauseAlignActive) {
    setMotorOutput(0);
    if (pauseServoHeld < 0) {
      pauseServoHeld = servoWithOffset(avoidDirectionRight, avoidServoOffset);
    }
    writeServoAngle(pauseServoHeld);
    finalServoAngle = pauseServoHeld;
    return;
  }
  pauseServoHeld = -1;
  int speed = constrain(avoidDriveSpeed, AVOID_SPEED_MIN, AVOID_SPEED_MAX);
  setMotorOutput(speed);
  int angle;
  if (holdPathActive) {
    // HOLD the line, but never let the outer-wall helper steer into the cube.
    float target = pathHeadingCaptured
        ? pathHeadingBeforeBlock
        : snapToCardinal(straightTargetHeading);
    angle = passSafeServo(steerTowardHeading(target, currentHeading));
  } else {
    angle = servoWithOffset(avoidDirectionRight, avoidServoOffset);
    angle = passSafeServo(angle);
  }
  writeServoAngle(angle);
  finalServoAngle = angle;
}

void executeImuReverse(float currentHeading) {
  setMotorOutput(BACKWARD_SPEED);
  finalServoAngle = steerTowardHeading(reverseHoldHeading, currentHeading);
  steeringServo.write(finalServoAngle);

  unsigned long elapsed = millis() - reverseStartMs;
  bool farEnough = currentCenterDist > PRE_TURN_CLEAR_CM;
  bool timedOut = elapsed >= max(PRE_TURN_REVERSE_MS, OBSTACLE_TIMEOUT_MS);

  if (currentState == PRE_TURN_REVERSE) {
    if ((elapsed >= PRE_TURN_REVERSE_MS && farEnough) || elapsed >= (PRE_TURN_REVERSE_MS + 400UL)) {
      beginCornerTurn();
    }
    return;
  }

  if (reverseThenAvoid) {
    if (farEnough || elapsed >= CUBE_RETRY_REVERSE_MS) {
      currentState = OBSTACLE_AVOIDING;
      lastObstacleCmd = millis();
      avoidPassStartMs = millis();
      stuckSteerUsed = true;
      avoidServoOffset = DIFF;
      ignoreFrontLidarFor(AVOID_LIDAR_IGNORE_MS);
      Serial.println("TEL reverse done — grid pass at full offset");
    }
    return;
  }

  if (millis() - lastPiCmd > OBSTACLE_TIMEOUT_MS || timedOut) {
    resumeStraightDriving();
    Serial.println("TEL REVERSE timeout — resume");
  }
}

void executeRejoin(float currentHeading) {
  setMotorOutput(REJOIN_SPEED);
  float targetHeading = pathHeadingCaptured
      ? pathHeadingBeforeBlock
      : snapToCardinal(straightTargetHeading);

  if (currentLeftDist > 0 && currentRightDist > 0) {
    float sideErr = (float)currentLeftDist - (float)currentRightDist;
    float bias = constrain(sideErr * 0.35, -12.0, 12.0);
    // More space on the left → yaw right onto center (BNO increases to the right).
    targetHeading = wrapHeading(targetHeading + bias);
  }

  finalServoAngle = wallSafeServo(steerTowardHeading(targetHeading, currentHeading));
  steeringServo.write(finalServoAngle);

  bool headingClose = fabs(shortestAngleDiff(currentHeading, targetHeading)) <= RECENTER_HEADING_DEG;
  bool sidesOk = true;
  if (currentLeftDist > 0 && currentRightDist > 0) {
    sidesOk = abs(currentLeftDist - currentRightDist) <= REJOIN_BALANCE_CM;
  }
  if (headingClose && sidesOk) {
    if (rejoinBalancedStart == 0) rejoinBalancedStart = millis();
  } else {
    rejoinBalancedStart = 0;
  }
  unsigned long elapsed = millis() - rejoinStartMs;
  bool held = headingClose && sidesOk && (millis() - rejoinBalancedStart >= REJOIN_HOLD_MS);
  if (held || elapsed >= REJOIN_MAX_MS) {
    straightTargetHeading = snapToCardinal(targetHeading);
    resumeStraightDriving();
    Serial.println("TEL REJOIN lane center");
  }
}

void executeParkWait() {
  setMotorOutput(0);
  steeringServo.write(SERVO_CENTER);
  finalServoAngle = SERVO_CENTER;
  checkParkButton();
}

void executeParkReverse(float currentHeading) {
  (void)currentHeading;
  setMotorOutput(-PARK_SPEED);
  steeringServo.write(SERVO_CENTER);
  finalServoAngle = SERVO_CENTER;
  if (millis() - parkPhaseStartTime >= PARK_REVERSE_MS) {
    currentState = PARK_PHASE1;
    parkPhaseStartTime = millis();
    parkPhaseStartHeading = getSmoothedHeading();
  }
}

void executeParkPhase1(float currentHeading) {
  int crank = parkCrankServoAngle(!parkDirectionRight);
  setMotorOutput(PARK_SPEED);
  steeringServo.write(crank);
  finalServoAngle = crank;

  float diffToTarget = shortestAngleDiff(currentHeading, parkTargetHeading1);
  bool headingClose = fabs(diffToTarget) <= PARK_EXIT_THRESHOLD;
  float rotatedSoFar = fabs(shortestAngleDiff(currentHeading, parkPhaseStartHeading));
  bool rotationOverrun = rotatedSoFar >= (PARK_EXIT_ANGLE_DEG + PARK_ROTATION_MARGIN_DEG);
  bool safetyCap = (millis() - parkPhaseStartTime) >= PARK_SAFETY_CAP_MS;

  if (headingClose || rotationOverrun || safetyCap) {
    if (rotationOverrun) Serial.println("TEL PARK P1 rotation overrun");
    currentState = PARK_STRAIGHT_LEG;
    parkPhaseStartTime = millis();
  }
}

void executeParkStraightLeg() {
  setMotorOutput(PARK_SPEED);
  steeringServo.write(SERVO_CENTER);
  finalServoAngle = SERVO_CENTER;
  if (millis() - parkPhaseStartTime >= PARK_STRAIGHT_MS) {
    currentState = PARK_PHASE2;
    parkPhaseStartTime = millis();
    parkPhaseStartHeading = getSmoothedHeading();
  }
}

void executeParkPhase2(float currentHeading) {
  int crank = parkCrankServoAngle(parkDirectionRight);
  setMotorOutput(PARK_SPEED);
  steeringServo.write(crank);
  finalServoAngle = crank;

  float diffToTarget = shortestAngleDiff(currentHeading, parkTargetHeading2);
  bool headingClose = fabs(diffToTarget) <= PARK_EXIT_THRESHOLD;
  float rotatedSoFar = fabs(shortestAngleDiff(currentHeading, parkPhaseStartHeading));
  bool rotationOverrun = rotatedSoFar >= (PARK_EXIT_ANGLE_DEG + PARK_ROTATION_MARGIN_DEG);
  bool safetyCap = (millis() - parkPhaseStartTime) >= PARK_SAFETY_CAP_MS;

  if (headingClose || rotationOverrun || safetyCap) {
    if (rotationOverrun) Serial.println("TEL PARK P2 rotation overrun");
    beginLaneAfterParking();
  }
}

void noteTracking(bool passRight, int heightPx) {
  piTracking = true;
  trackedPassRight = passRight;
  trackedHeightPx = heightPx;
  lastTrackingMs = millis();
  lastPiCmd = millis();
}

void applyDriveCommand(String line) {
  String color = parsePiField(line, 1);
  color.toLowerCase();
  int speed = parsePiIntField(line, 2, AVOID_SPEED);
  int offset = parsePiIntField(line, 3, 18);
  String phase = parsePiField(line, 4);
  phase.toUpperCase();
  int height = parsePiIntField(line, 5, trackedHeightPx);

  if (!cubesEnabled || inParkManeuver()) {
    return;
  }

  bool passRight = (color == "red");
  noteTracking(passRight, height);
  avoidDirectionRight = passRight;
  holdPathActive = (phase == "HOLD");
  pauseAlignActive = (phase == "PAUSE");
  int requestedOffset = constrain(offset, 0, DIFF);
  int requestedSpeed;
  if (pauseAlignActive) {
    requestedSpeed = 0;
    stuckSteerUsed = false;
    stuckReverseUsed = false;
  } else {
    requestedSpeed = constrain(speed, AVOID_SPEED_MIN, AVOID_SPEED_MAX);
  }
  if (holdPathActive) {
    stuckSteerUsed = false;
    stuckReverseUsed = false;
    avoidServoOffset = requestedOffset;
    avoidDriveSpeed = requestedSpeed;
  } else if (stuckSteerUsed) {
    avoidServoOffset = DIFF;
    avoidDriveSpeed = min(requestedSpeed, AVOID_STUCK_SPEED);
  } else {
    avoidServoOffset = requestedOffset;
    avoidDriveSpeed = requestedSpeed;
  }

  if (inParkManeuver() || currentState == TURNING || currentState == PRE_TURN_REVERSE ||
      currentState == ROBOT_STOPPED) {
    return;
  }
  if (currentState == PI_REVERSE && reverseThenAvoid) {
    if (millis() - reverseStartMs < CUBE_RETRY_REVERSE_MS) {
      return;
    }
  }

  capturePathHeadingBeforeBlock();
  ignoreFrontLidarFor(AVOID_LIDAR_IGNORE_MS);

  if (phase == "SIDE") {
    pauseAlignActive = false;
    pauseServoHeld = -1;
    stuckSteerUsed = false;
    stuckReverseUsed = false;
    avoidDriveSpeed = requestedSpeed;
    if (currentState == PI_YAW_BACK || currentState == PI_REJOIN) {
      lastObstacleCmd = millis();
      return;
    }
    if (currentState != PI_SIDE) {
      beginSideWait();
    } else {
      lastObstacleCmd = millis();
    }
    return;
  }

  if (phase == "RECOVER" || phase == "REJOIN") {
    if (currentState == PI_SIDE || currentState == PI_YAW_BACK) {
      return;
    }
    stuckSteerUsed = false;
    stuckReverseUsed = false;
    if (currentState != PI_REJOIN) {
      rejoinStartMs = millis();
      rejoinBalancedStart = 0;
      currentState = PI_REJOIN;
      Serial.println("TEL DRIVE REJOIN");
    }
    return;
  }

  if (currentState != OBSTACLE_AVOIDING) {
    avoidPassStartMs = millis();
    stuckSteerUsed = false;
    stuckReverseUsed = false;
  } else if (!pauseAlignActive && pauseServoHeld >= 0) {
    avoidPassStartMs = millis();
    stuckSteerUsed = false;
    stuckReverseUsed = false;
  }
  currentState = OBSTACLE_AVOIDING;
  lastObstacleCmd = millis();
}

void handlePiReverse() {
  lastPiCmd = millis();
  if (!cubesEnabled || inParkManeuver() || currentState == TURNING || currentState == PRE_TURN_REVERSE ||
      currentState == ROBOT_STOPPED || currentState == PI_REVERSE || inSideOrYaw()) {
    return;
  }
  capturePathHeadingBeforeBlock();
  reverseHoldHeading = snapToCardinal(
      pathHeadingCaptured ? pathHeadingBeforeBlock : getSmoothedHeading());
  reverseStartMs = millis();
  reverseThenAvoid = (currentState == OBSTACLE_AVOIDING) || trackingFresh();
  currentState = PI_REVERSE;
  ignoreFrontLidarFor(AVOID_LIDAR_IGNORE_MS);
  Serial.println("TEL PI REVERSE cardinal");
}

void handlePiClear() {
  piTracking = false;
  trackedHeightPx = 0;
  if (inParkManeuver() || currentState == TURNING || currentState == PRE_TURN_REVERSE ||
      currentState == ROBOT_STOPPED || inSideOrYaw()) {
    return;
  }
  if (currentState == OBSTACLE_AVOIDING || currentState == PI_REVERSE) {
    stuckSteerUsed = false;
    stuckReverseUsed = false;
    rejoinStartMs = millis();
    rejoinBalancedStart = 0;
    currentState = PI_REJOIN;
    ignoreFrontLidarFor(AVOID_LIDAR_IGNORE_MS);
    Serial.println("TEL CLEAR — rejoin center");
    return;
  }
  if (currentState == PI_REJOIN) {
    return;
  }
}

void handlePiHello() {
  if (!handshakeArmed) {
    return;
  }
  piLinkOk = true;
  Serial.println("ESP_ACK");
}

void handlePiLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (line.startsWith("TEL")) return;

  if (!lineChecksumOk(line)) {
    Serial.println("ESP_NAK");
    return;
  }

  int star = line.indexOf('*');
  if (star >= 0) line = line.substring(0, star);
  line.trim();

  int comma = line.indexOf(',');
  String name = (comma < 0) ? line : line.substring(0, comma);
  name.trim();
  name.toUpperCase();

  if (name == "PI_HELLO" || name == "HELLO") {
    handlePiHello();
    return;
  }

  if (name != "DRIVE" && name != "REVERSE" && name != "CLEAR" &&
      name != "RED" && name != "GREEN" && name != "TRACKING") {
    Serial.println("ESP_NAK");
    resetPiSerialRx();
    return;
  }

  if (!piLinkOk) {
    if (handshakeArmed) Serial.println("ESP_READY");
    return;
  }

  if (name == "DRIVE") {
    applyDriveCommand(line);
  } else if (name == "TRACKING") {
    int on = parsePiIntField(line, 1, 1);
    int height = parsePiIntField(line, 2, 0);
    String color = parsePiField(line, 3);
    color.toLowerCase();
    if (on == 0) {
      piTracking = false;
      trackedHeightPx = 0;
    } else if (cubesEnabled) {
      noteTracking(color != "green", height);
    }
  } else if (name == "REVERSE") {
    handlePiReverse();
  } else if (name == "CLEAR") {
    handlePiClear();
  } else if (name == "RED") {
    if (!cubesEnabled || inParkManeuver() || currentState == TURNING || currentState == PRE_TURN_REVERSE) return;
    avoidDirectionRight = true;
    avoidServoOffset = constrain(parsePiIntField(line, 1, 18), 0, DIFF);
    avoidDriveSpeed = AVOID_SPEED;
    noteTracking(true, parsePiIntField(line, 2, trackedHeightPx));
    capturePathHeadingBeforeBlock();
    currentState = OBSTACLE_AVOIDING;
    lastObstacleCmd = millis();
    ignoreFrontLidarFor(AVOID_LIDAR_IGNORE_MS);
  } else if (name == "GREEN") {
    if (!cubesEnabled || inParkManeuver() || currentState == TURNING || currentState == PRE_TURN_REVERSE) return;
    avoidDirectionRight = false;
    avoidServoOffset = constrain(parsePiIntField(line, 1, 18), 0, DIFF);
    avoidDriveSpeed = AVOID_SPEED;
    noteTracking(false, parsePiIntField(line, 2, trackedHeightPx));
    capturePathHeadingBeforeBlock();
    currentState = OBSTACLE_AVOIDING;
    lastObstacleCmd = millis();
    ignoreFrontLidarFor(AVOID_LIDAR_IGNORE_MS);
  }
}

void advertiseLink() {
  if (!handshakeArmed || piLinkOk) return;
  if (millis() - lastReadyAdvert >= ESP_READY_PERIOD_MS) {
    lastReadyAdvert = millis();
    Serial.println("ESP_READY");
  }
}

void printTelemetry(float currentHeading) {
  Serial.print("TEL ");
  if (currentState == DRIVING_STRAIGHT) Serial.print("STRAIGHT");
  else if (currentState == TURNING) {
    Serial.print(currentTurnPhase == PHASE_PAUSE ? "PAUSE" : "ARC");
  }
  else if (currentState == OBSTACLE_AVOIDING) Serial.print("AVOID");
  else if (currentState == PI_SIDE)           Serial.print("SIDE");
  else if (currentState == PI_YAW_BACK)       Serial.print("YAW");
  else if (currentState == PI_REVERSE)        Serial.print("REVERSE");
  else if (currentState == PRE_TURN_REVERSE)  Serial.print("PRETURN");
  else if (currentState == PI_REJOIN)         Serial.print("REJOIN");
  else if (currentState == PARK_WAIT)         Serial.print("PARKWAIT");
  else if (currentState == PARK_REVERSE)      Serial.print("PARKREV");
  else if (currentState == PARK_PHASE1)       Serial.print("PARKP1");
  else if (currentState == PARK_STRAIGHT_LEG) Serial.print("PARKSTR");
  else if (currentState == PARK_PHASE2)       Serial.print("PARKP2");
  Serial.print(" slot="); Serial.print(parkDirectionRight ? "R" : "L");

  Serial.print(" L="); Serial.print(currentLeftDist);
  Serial.print(" C="); Serial.print(currentCenterDist);
  Serial.print(" R="); Serial.print(currentRightDist);
  Serial.print(" turns="); Serial.print(totalTurnsCount);
  Serial.print("/"); Serial.print(MAX_TURNS);
  Serial.print(" cubes="); Serial.print(cubesEnabled ? 1 : 0);
  Serial.print(" blocks<= "); Serial.print(MAX_BLOCKS);
  Serial.print(" link="); Serial.print(piLinkOk ? 1 : 0);
  Serial.print(" trk="); Serial.print(trackingFresh() ? 1 : 0);
  Serial.print(" hpx="); Serial.print(trackedHeightPx);
  Serial.print(" H="); Serial.print(currentHeading);
  Serial.print(" servo="); Serial.println(finalServoAngle);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  resetPiSerialRx();
  Wire.begin(21, 22);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(PARK_BUTTON_PIN, INPUT_PULLUP);
  pinMode(READY_LED_PIN, OUTPUT);
  digitalWrite(READY_LED_PIN, LOW);

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);

  selectMuxChannel(MUX_CH_BNO);
  if (!bno.begin()) {
    Serial.println("TEL Critical: BNO055 missing on mux ch 4");
    while (1) { delay(200); }
  }
  delay(500);
  bno.setExtCrystalUse(true);

  Serial.println("TEL Waiting for BNO055 calibration...");
  uint8_t sysCal, gyroCal, accelCal, magCal;
  unsigned long calStart = millis();
  do {
    selectMuxChannel(MUX_CH_BNO);
    bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);
    Serial.print("TEL Cal Sys:"); Serial.print(sysCal);
    Serial.print(" Gyro:"); Serial.print(gyroCal);
    Serial.print(" Accel:"); Serial.print(accelCal);
    Serial.print(" Mag:"); Serial.println(magCal);
    delay(200);
  } while (gyroCal < 3 && millis() - calStart < 10000);
  Serial.println("TEL Calibration wait done.");

  straightTargetHeading = snapToCardinal(getCurrentHeading());
  driveStartTime = millis();
  rampActive = false;
  systemReady = true;
  digitalWrite(READY_LED_PIN, HIGH);
  currentState = PARK_WAIT;
  Serial.println("TEL READY — press button on pin 32 to start handshake + parking exit");
}

void resetPiSerialRx() {
  while (Serial.available()) {
    Serial.read();
  }
}

void pollPiSerial() {
  static String serialBuffer = "";
  static unsigned long lastRxMs = 0;
  const unsigned int MAX_LINE = 96;
  const unsigned long STALE_MS = 200;

  while (Serial.available()) {
    char c = (char)Serial.read();
    lastRxMs = millis();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        handlePiLine(serialBuffer);
      }
      serialBuffer = "";
    } else if (c < 32 || c > 126) {
      serialBuffer = "";
      Serial.println("ESP_NAK");
    } else if (serialBuffer.length() >= MAX_LINE) {
      serialBuffer = "";
      Serial.println("ESP_NAK");
    } else {
      serialBuffer += c;
    }
  }

  if (serialBuffer.length() > 0 && (millis() - lastRxMs >= STALE_MS)) {
    serialBuffer = "";
  }
}

void trackTurnsAndStop() {
  if (inParkManeuver()) {
    stopConditionActive = false;
    return;
  }
  bool stopConditionMet =
      (currentLeftDist   > 0 && currentLeftDist   < 100) &&
      (currentRightDist  > 0 && currentRightDist  < 100) &&
      (currentCenterDist > 0 && currentCenterDist < 150) &&
      (totalTurnsCount >= MAX_TURNS);

  if (stopConditionMet) {
    if (!stopConditionActive) {
      stopConditionActive = true;
      stopConditionStartTime = millis();
    } else if (millis() - stopConditionStartTime >= STOP_CONFIRM_MS) {
      currentState = ROBOT_STOPPED;
    }
  } else {
    stopConditionActive = false;
  }
}

void loop() {
  advertiseLink();
  pollPiSerial();

  if (!trackingFresh() && piTracking && (millis() - lastTrackingMs > TRACKING_HOLD_MS)) {
    piTracking = false;
  }

  if (currentState == OBSTACLE_AVOIDING && trackingFresh() && !holdPathActive && !pauseAlignActive) {
    unsigned long passMs = millis() - avoidPassStartMs;
    if (!stuckSteerUsed && passMs >= PASS_STUCK_MS) {
      stuckSteerUsed = true;
      avoidServoOffset = DIFF;
      Serial.println("TEL PASS still not clear — servo full offset");
    }
    if (!stuckReverseUsed && passMs >= PASS_STUCK_REVERSE_MS) {
      stuckReverseUsed = true;
      beginCubeReverseThenAvoid();
      Serial.println("TEL PASS still not clear — reverse and retry");
    }
  }

  if (currentState == OBSTACLE_AVOIDING && (millis() - lastObstacleCmd > OBSTACLE_TIMEOUT_MS)) {
    rejoinStartMs = millis();
    rejoinBalancedStart = 0;
    currentState = PI_REJOIN;
    Serial.println("TEL AVOID timeout — rejoin");
  }

  if (currentState == OBSTACLE_AVOIDING && millis() >= turnCooldownUntil &&
      currentCenterDist > 0 &&
      currentCenterDist < FRONT_TURN_DISTANCE && trackingFresh() &&
      trackedHeightPx >= BLOCK_IN_FRONT_PX) {
    beginCubeReverseThenAvoid();
  }

  if (currentState == ROBOT_STOPPED) {
    setMotorOutput(0);
    steeringServo.write(SERVO_CENTER);
    Serial.print("TEL STATUS Finished turns=");
    Serial.println(totalTurnsCount);
    delay(200);
    return;
  }

  currentLeftDist   = getLunaDistance(MUX_CH_LEFT);
  currentCenterDist = getLunaDistance(MUX_CH_CENTER);
  currentRightDist  = getLunaDistance(MUX_CH_RIGHT);
  float currentHeading = getSmoothedHeading();

  trackTurnsAndStop();
  if (currentState == ROBOT_STOPPED) {
    return;
  }

  if (currentState == DRIVING_STRAIGHT) {
    checkFrontObstacle();
  }

  if (currentState == DRIVING_STRAIGHT) {
    driveStraightMode(currentHeading);
  }
  else if (currentState == TURNING) {
    executeTurnMode(currentHeading);
  }
  else if (currentState == OBSTACLE_AVOIDING) {
    avoidObstacle(currentHeading);
  }
  else if (currentState == PI_SIDE) {
    executeSideWait(currentHeading);
  }
  else if (currentState == PI_YAW_BACK) {
    executeYawBack(currentHeading);
  }
  else if (currentState == PI_REVERSE || currentState == PRE_TURN_REVERSE) {
    executeImuReverse(currentHeading);
  }
  else if (currentState == PI_REJOIN) {
    executeRejoin(currentHeading);
  }
  else if (currentState == PARK_WAIT) {
    executeParkWait();
  }
  else if (currentState == PARK_REVERSE) {
    executeParkReverse(currentHeading);
  }
  else if (currentState == PARK_PHASE1) {
    executeParkPhase1(currentHeading);
  }
  else if (currentState == PARK_STRAIGHT_LEG) {
    executeParkStraightLeg();
  }
  else if (currentState == PARK_PHASE2) {
    executeParkPhase2(currentHeading);
  }

  unsigned long nowTel = millis();
  if (nowTel - lastTelemetryMs >= TELEMETRY_MS) {
    lastTelemetryMs = nowTel;
    printTelemetry(currentHeading);
  }
  delay(20);
}
