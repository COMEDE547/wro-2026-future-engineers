#include <Wire.h>
#include <Adafruit_Sensor.h>
#define ENABLE_BLUETOOTH 0   // rule 11.10: radios must be OFF during rounds — leave 0 for competition builds
#if ENABLE_BLUETOOTH
#include "BluetoothSerial.h"
#endif
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>

#define SERVO_PIN 13

#define MOTOR_IN1  25
#define MOTOR_IN2  26
#define MOTOR_PWM  33
#define MOTOR_STBY 27        // TB6612 STBY: driven HIGH; harmless if STBY is hardwired high on the breakout

#define START_BUTTON_PIN 0   // onboard BOOT button; move to external button pin when mounted (rules 9.10-9.11)

#define MUX_CH_LEFT   0
#define MUX_CH_CENTER 1  
#define MUX_CH_RIGHT  2
#define MUX_CH_BNO    4

#define TFLUNA_I2C_ADDR 0x10

float SLOW_STEER_THRESHOLD = 50.0; // Angle (in degrees) where the servo starts returning to center
int STRAIGHT_SPEED = 100;  // Cruise speed for driving straight (0-255)
int TURN_SPEED     = 100;  // Controlled speed for turning to prevent overshooting
int BACKWARD_SPEED = -100; // Speed for backing up (if needed)


int SERVO_CENTER   = 106;   // Dead-center steering alignment — aligned 2026-08-10 to Round 1's mat-tuned value (58adb1c); same physical linkage, chassis final
int SERVO_MAX_LEFT  = 136;  // Physical linkage limit, higher-angle side (asymmetric about center: +30)
int SERVO_MAX_RIGHT = 64;   // Physical linkage limit, lower-angle side  (asymmetric about center: -42)
int DIFF = 42;              // Visual-swerve offset cap = the larger center-to-limit span; the per-side
                            // constrain(SERVO_MAX_RIGHT, SERVO_MAX_LEFT) below truncates the shorter (+30) side.
                            // 2026-08-10: was SERVO_CENTER±DIFF(35) symmetric — could not express the real
                            // 64/136 window (106+35=141 would command past the linkage). NOT yet mat-tested
                            // at these values: bench-verify steering direction (INVERT_STEERING) and center
                            // before flashing for a run.

// Change to true if your steering corrections move backwards during testing
const bool INVERT_STEERING = true;


const float STEERING_KP  = 1.2;   // Proportional gain — how hard it steers based on current error
const float STEERING_KD  = 0.15 ;   // Derivative gain — how hard it "brakes" based on how fast error is changing
const float HEADING_DEADBAND = 1.5;   // degrees — ignore jitter smaller than this
const int   MAX_STEER_CORRECTION = 20; // clamp — no single correction can swing the servo too hard
const int SPIKE_THRESHOLD = 100;  // Trigger turning mode if sensor difference changes by this many cm
const int MAX_TURNS      = 12;   // Total number of turns allowed before tracking the final stop distance

unsigned long turnCooldownUntil = 0;   // NEW: timestamp until which obstacle checks are ignored
const unsigned long TURN_COOLDOWN_MS = 300; // tune this — how long to ignore after a turn
const int SPIKE_PERSIST_LOOPS = 5;     // corner spike must hold this many consecutive loops (rejects pillar occlusion)

// gradient visual steering (tune on mat)
const float KV_VISUAL = 0.28f;         // deg of steer per px of visual error
const int   MIN_ACTIVE_SWERVE = 8;     // deg floor while error > 0, guarantees progress
const int   SAFE_RED_X   = 90;         // block safe when cx <= this (matches Pi LEFT_SIDE_MAX)
const int   SAFE_GREEN_X = 150;        // block safe when cx >= this (matches Pi RIGHT_SIDE_MIN)

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo steeringServo;
#if ENABLE_BLUETOOTH
BluetoothSerial SerialBT;    // Added Bluetooth object
#endif

enum RobotState { WAIT_FOR_START, DRIVING_STRAIGHT, TURNING, ROBOT_STOPPED, OBSTACLE_AVOIDING, REVERSING };
RobotState currentState = WAIT_FOR_START;

bool avoidDirectionRight = true;   // true = swerve right (for RED), false = swerve left (for GREEN)
unsigned long lastObstacleCmd = 0; // timeout safety
const unsigned long OBSTACLE_TIMEOUT_MS = 1500; // dead-man: auto-clear if no command for 1.5 s (Pi keeps state alive every 0.5 s)
unsigned long lastReverseCmd = 0;  // last time a REVERSE arrived
int lastPosX = -1;                 // latest block center-x from the Pi POS stream (0..239)
unsigned long lastPosMs = 0;       // when it arrived; 0 = none received this avoidance

float straightTargetHeading = 0.0;
float turnTargetHeading     = 0.0;
bool isTurningLeft          = false;
int totalTurnsCount         = 0;
bool hasTurnedOnce          = false; // Becomes true permanently on the first turn
bool lockedDirectionLeft    = false; // Remembers if our layout is strictly Left or Right

int16_t currentLeftDist     = -1;
int16_t currentCenterDist   = -1; 
int16_t currentRightDist    = -1;
int finalServoAngle         = SERVO_CENTER;  // start at center (2026-08-10: was 90 ≠ center — latent init smell)
float headingError          = 0.0;
float angleDifference       = 0.0;
float lastHeadingError       = 0.0;   // NEW: needed to calculate the D term
unsigned long lastHeadingTime = 0;    // NEW: for time-based derivative

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

float getSmoothedHeading() {
  float raw = getCurrentHeading();
  if (!headingInitialized) {
    filteredHeading = raw;
    headingInitialized = true;
    return filteredHeading;
  }

  // Find the shortest angular distance from filtered to raw (handles the 0/360 wrap)
  float diff = raw - filteredHeading;
  if (diff > 180.0)  diff -= 360.0;
  if (diff < -180.0) diff += 360.0;

  filteredHeading += 0.2 * diff;   // same 0.8/0.2 blend, but wrap-safe
  if (filteredHeading < 0.0)    filteredHeading += 360.0;
  if (filteredHeading >= 360.0) filteredHeading -= 360.0;

  return filteredHeading;
}



template <typename T>
void btPrint(T msg) {
  Serial.print(msg);
#if ENABLE_BLUETOOTH
  SerialBT.print(msg);
#endif
}


template <typename T>
void btPrintln(T msg) {
  Serial.println(msg);
#if ENABLE_BLUETOOTH
  SerialBT.println(msg);
#endif
}


float wrap360(float a) {
  a = fmod(a, 360.0);
  if (a < 0.0) a += 360.0;
  return a;
}

// Step each turn target from the PREVIOUS target, not the current heading:
// legs stay exactly 90 degrees apart in ANY IMU reference frame and drift cannot accumulate.
float computeTurnTarget(bool turningLeft) {
  return wrap360(straightTargetHeading + (turningLeft ? -90.0 : 90.0));
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


void checkObstacles() {
  static int spikeCount = 0;
  static int lastSpikeDir = 0;   // +1 = left-turn spike, -1 = right-turn spike

  if (millis() < turnCooldownUntil) { spikeCount = 0; return; }
  if (currentLeftDist == -1 || currentRightDist == -1) return;

  int16_t difference = currentLeftDist - currentRightDist;

  bool spikeLeft  = (difference >  SPIKE_THRESHOLD);
  bool spikeRight = (difference < -SPIKE_THRESHOLD);

  // After the first turn the layout direction is locked: only that direction can arm.
  if (hasTurnedOnce) {
    if (lockedDirectionLeft) spikeRight = false;
    else                     spikeLeft  = false;
  }

  if (!spikeLeft && !spikeRight) { spikeCount = 0; return; }

  // Corner spike must persist SPIKE_PERSIST_LOOPS consecutive loops in the SAME direction —
  // a pillar momentarily occluding one side Luna cannot fake a corner.
  int dirNow = spikeLeft ? 1 : -1;
  if (dirNow != lastSpikeDir) spikeCount = 0;
  lastSpikeDir = dirNow;
  spikeCount++;
  if (spikeCount < SPIKE_PERSIST_LOOPS) return;
  spikeCount = 0;

  isTurningLeft = spikeLeft;
  turnTargetHeading = computeTurnTarget(isTurningLeft);

  if (!hasTurnedOnce) {
    hasTurnedOnce = true;
    lockedDirectionLeft = isTurningLeft;
    if (isTurningLeft) btPrintln("!!! LAYOUT INITIALIZED: PERMANENT LEFT TURN ONLY LOCK ACTIVATED !!!");
    else               btPrintln("!!! LAYOUT INITIALIZED: PERMANENT RIGHT TURN ONLY LOCK ACTIVATED !!!");
  }

  currentState = TURNING;
}

void avoidObstacle() {

  setMotorOutput(STRAIGHT_SPEED);  // or a slightly slower speed

  // GRADIENT VISUAL STEERING: steer proportionally to how far the block sits from its
  // pass-side safe line in the Pi's 240-px frame. The old binary full-lock swerve is the
  // saturation clamp of this law (large error => offset = DIFF => identical behavior).
  if (lastPosMs == 0) {
    // No POS received this avoidance yet — fall back to the field-proven full lock.
    int swerveAngle;
    if (avoidDirectionRight) {
      swerveAngle = INVERT_STEERING ? SERVO_MAX_LEFT : SERVO_MAX_RIGHT;
    } else {
      swerveAngle = INVERT_STEERING ? SERVO_MAX_RIGHT : SERVO_MAX_LEFT;
    }
    steeringServo.write(swerveAngle);
    finalServoAngle = swerveAngle;
    return;
  }

  if (millis() - lastPosMs > 300) {
    // POS stream stale — hold the last commanded angle (no snap to center, no full lock).
    steeringServo.write(finalServoAngle);
    return;
  }

  int error;
  if (avoidDirectionRight) {
    error = lastPosX - SAFE_RED_X;      // RED: push the block left of the red safe line
  } else {
    error = SAFE_GREEN_X - lastPosX;    // GREEN: push the block right of the green safe line
  }
  int offset = constrain((int)(KV_VISUAL * error), 0, DIFF);
  if (error > 0 && offset < MIN_ACTIVE_SWERVE) offset = MIN_ACTIVE_SWERVE;

  int swerveAngle;
  if (avoidDirectionRight) {
    swerveAngle = INVERT_STEERING ? (SERVO_CENTER + offset) : (SERVO_CENTER - offset);
  } else {
    swerveAngle = INVERT_STEERING ? (SERVO_CENTER - offset) : (SERVO_CENTER + offset);
  }
  swerveAngle = constrain(swerveAngle, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
  steeringServo.write(swerveAngle);
  finalServoAngle = swerveAngle;
}




void driveStraightMode(float currentHeading) {
  setMotorOutput(STRAIGHT_SPEED);

  float rawError = straightTargetHeading - currentHeading;
  if (rawError > 180.0)  rawError -= 360.0;
  if (rawError < -180.0) rawError += 360.0;

  headingError = rawError;

  unsigned long now = millis();
  float dt = (now - lastHeadingTime) / 1000.0;
  if (dt < 0.001) dt = 0.001;

  float errorRate = (rawError - lastHeadingError) / dt;

  float pTermInput = (abs(rawError) < HEADING_DEADBAND) ? 0.0 : rawError;

  int steeringCorrection = (int)(pTermInput * STEERING_KP + errorRate * STEERING_KD);
  steeringCorrection = constrain(steeringCorrection, -MAX_STEER_CORRECTION, MAX_STEER_CORRECTION);

  lastHeadingError = rawError;
  lastHeadingTime = now;

  if (INVERT_STEERING) {
    finalServoAngle = SERVO_CENTER + steeringCorrection;
  } else {
    finalServoAngle = SERVO_CENTER - steeringCorrection;
  }
  finalServoAngle = constrain(finalServoAngle, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
  steeringServo.write(finalServoAngle);
}


void executeTurnMode(float currentHeading) {
  currentHeading = getCurrentHeading();  // A11: raw heading here — the EMA's ~200 ms lag would delay completion and overshoot the corner
  setMotorOutput(TURN_SPEED);


  angleDifference = currentHeading - turnTargetHeading;
  if (angleDifference > 180.0)  angleDifference -= 360.0;
  if (angleDifference < -180.0) angleDifference += 360.0;
  float remainingAngle = abs(angleDifference);


  int leftExtreme  = INVERT_STEERING ? SERVO_MAX_RIGHT : SERVO_MAX_LEFT;
  int rightExtreme = INVERT_STEERING ? SERVO_MAX_LEFT  : SERVO_MAX_RIGHT;

  if (remainingAngle < SLOW_STEER_THRESHOLD) {
    float progressFactor = remainingAngle / SLOW_STEER_THRESHOLD;
    if (isTurningLeft) {
      int maxOffset = leftExtreme - SERVO_CENTER;
      finalServoAngle = SERVO_CENTER + (int)(maxOffset * progressFactor);
    } else {
      int maxOffset = SERVO_CENTER - rightExtreme;
      finalServoAngle = SERVO_CENTER - (int)(maxOffset * progressFactor);
    }
  } else {
    finalServoAngle = isTurningLeft ? leftExtreme : rightExtreme;
  }


  steeringServo.write(finalServoAngle);


  if (remainingAngle < 6.0) {
  totalTurnsCount++;
  steeringServo.write(SERVO_CENTER);
  finalServoAngle = SERVO_CENTER;
  straightTargetHeading = turnTargetHeading;
  currentState = DRIVING_STRAIGHT;
  turnCooldownUntil = millis() + TURN_COOLDOWN_MS;   // NEW
  }
}


void printTelemetry(float currentHeading) {
  if (currentState == DRIVING_STRAIGHT) btPrint("MODE: STRAIGHT");
  else if (currentState == TURNING)     btPrint("MODE: TURNING ");
  else if (currentState == OBSTACLE_AVOIDING) btPrint("MODE: AVOIDING");
  else if (currentState == REVERSING)   btPrint("MODE: REVERSING");


  btPrint(" | L: "); btPrint(currentLeftDist); btPrint("cm");
  btPrint(" | C: "); btPrint(currentCenterDist); btPrint("cm");
  btPrint(" | R: "); btPrint(currentRightDist); btPrint("cm");
  btPrint(" | Turns: "); btPrint(totalTurnsCount);


  if (hasTurnedOnce) {
    btPrint(" | LOCK: "); btPrint(lockedDirectionLeft ? "LEFT_ONLY" : "RIGHT_ONLY");
  } else {
    btPrint(" | LOCK: NONE");
  }


  if (currentState == DRIVING_STRAIGHT) {
    btPrint(" | Target: "); btPrint(straightTargetHeading);
    btPrint("° | Current: "); btPrint(currentHeading);
    btPrint("° | Error: "); btPrint(headingError);
    btPrint("°");
  } else if (currentState == TURNING) {
    btPrint(" | Target: "); btPrint(turnTargetHeading);
    btPrint("° | Current: "); btPrint(currentHeading);
    btPrint("° | Delta: "); btPrint(abs(angleDifference));
    btPrint("°");
  }


  btPrint(" | Servo Angle: "); btPrint(finalServoAngle); btPrintln("°");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
#if ENABLE_BLUETOOTH
  SerialBT.begin("ESP32_Robot_Telemetry");
#endif

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);   // TB6612 out of standby; harmless if STBY is hardwired high

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);  // rules 9.10-9.11: wait for one start button

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);


  selectMuxChannel(MUX_CH_BNO);
  // IMUPLUS: gyro+accel fusion, no magnetometer — boot yaw reads ~0 and motor magnets cannot distort heading
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS)) {
    btPrintln("Critical Error: BNO055 missing on Multiplexer channel 4!");
  }
  else {
    btPrintln("BNO055 OK on multiplexer channel 4 (IMUPLUS mode)");
    // NO while(1) on failure above — robot still limps for bench diagnosis
  }
  // delay(500);
  // bno.setExtCrystalUse(true);

  // Heading is NOT captured here — it is captured at the start-button press (WAIT_FOR_START),
  // after the robot has been placed on the mat.
  btPrintln("=== ROBOT READY - press START button ===");
  btPrintln("Commands: RED, GREEN, CLEAR, REVERSE, POS");
}


void loop() {

static String serialBuffer = "";
while (Serial.available()) {
  char c = Serial.read();
  if (c == '\n') {

    serialBuffer.trim();
    // State-gated protocol: commands are only honored where they are safe.
    //   RED/GREEN : STRAIGHT + AVOIDING (a mid-avoid color switch MUST flip the swerve)
    //   CLEAR     : AVOIDING + REVERSING
    //   REVERSE   : STRAIGHT + AVOIDING + REVERSING
    //   POS,cx,h  : STRAIGHT + AVOIDING + REVERSING (tracking data)
    //   Everything is ignored during TURNING (never abort a corner), ROBOT_STOPPED
    //   (a post-finish detection must never restart the robot), and WAIT_FOR_START.
    if (serialBuffer == "RED") {
      if (currentState == DRIVING_STRAIGHT || currentState == OBSTACLE_AVOIDING) {
        if (currentState == DRIVING_STRAIGHT) lastPosMs = 0;  // fresh pillar: full lock until first POS
        avoidDirectionRight = true;
        currentState = OBSTACLE_AVOIDING;
        Serial.println("OBSTACLE: RED - steering right of the block");
        lastObstacleCmd = millis();
      }
    } 
    else if (serialBuffer == "GREEN") {
      if (currentState == DRIVING_STRAIGHT || currentState == OBSTACLE_AVOIDING) {
        if (currentState == DRIVING_STRAIGHT) lastPosMs = 0;  // fresh pillar: full lock until first POS
        avoidDirectionRight = false;
        currentState = OBSTACLE_AVOIDING;
        Serial.println("OBSTACLE: GREEN - steering left of the block");
        lastObstacleCmd = millis();
      }
    } 
    else if (serialBuffer == "CLEAR") {
      if (currentState == OBSTACLE_AVOIDING || currentState == REVERSING) {
        currentState = DRIVING_STRAIGHT;

        lastHeadingError = 0;
        lastHeadingTime = millis();
        Serial.println("OBSTACLE: CLEARED - Returning to path");
      }
    }
    else if (serialBuffer == "REVERSE") {
      if (currentState == DRIVING_STRAIGHT || currentState == OBSTACLE_AVOIDING || currentState == REVERSING) {
        if (currentState != REVERSING) Serial.println("OBSTACLE: TOO CLOSE - backing up");
        currentState = REVERSING;
        lastReverseCmd = millis();
        lastObstacleCmd = millis();
      }
    }
    else if (serialBuffer.startsWith("POS,")) {
      if (currentState == DRIVING_STRAIGHT || currentState == OBSTACLE_AVOIDING || currentState == REVERSING) {
        int c1 = serialBuffer.indexOf(',', 4);
        if (c1 > 4) {
          lastPosX = serialBuffer.substring(4, c1).toInt();
          lastPosMs = millis();
          lastObstacleCmd = millis();   // the POS stream doubles as the keepalive
        }
      }
    }
    serialBuffer = "";
  } else {
    serialBuffer += c;
    if (serialBuffer.length() > 32) serialBuffer = "";  // noise hardening
  }
}

if ((currentState == OBSTACLE_AVOIDING || currentState == REVERSING) &&
    (millis() - lastObstacleCmd > OBSTACLE_TIMEOUT_MS)) {
  currentState = DRIVING_STRAIGHT;
  lastHeadingError = 0;
  lastHeadingTime = millis();
  Serial.println("OBSTACLE: Dead-man timeout - link or tracker silent, returning to path");
}

  if (currentState == WAIT_FOR_START) {
    setMotorOutput(0);
    steeringServo.write(SERVO_CENTER);
    // Debounced start button (active LOW, 50 ms). Heading is captured HERE, with the
    // robot placed on the mat, so the reference frame is the field, not the workbench.
    static unsigned long pressStart = 0;
    if (digitalRead(START_BUTTON_PIN) == LOW) {
      if (pressStart == 0) pressStart = millis();
      else if (millis() - pressStart >= 50) {
        straightTargetHeading = getCurrentHeading();
        lastHeadingTime = millis();
        lastHeadingError = 0;
        currentState = DRIVING_STRAIGHT;
        btPrintln("=== START ===");
      }
    } else {
      pressStart = 0;
    }
    delay(10);
    return;
  }

  if (currentState == ROBOT_STOPPED) {
    setMotorOutput(0);
    steeringServo.write(SERVO_CENTER);
   
    btPrint("STATUS: Finished. Total Turns Executed: ");
    btPrintln(totalTurnsCount);
   
    delay(1000);
    return;
  }


  currentLeftDist   = getLunaDistance(MUX_CH_LEFT);
  currentCenterDist = getLunaDistance(MUX_CH_CENTER);
  currentRightDist  = getLunaDistance(MUX_CH_RIGHT);


  float currentHeading = getSmoothedHeading();


  if (totalTurnsCount >= MAX_TURNS && currentCenterDist > 0 && currentCenterDist < 165) {
    currentState = ROBOT_STOPPED;
    return;
  }


  if (currentState == DRIVING_STRAIGHT) {
    checkObstacles();
    driveStraightMode(currentHeading);
  }

  else if (currentState == OBSTACLE_AVOIDING) {
  avoidObstacle();
}
  else if (currentState == REVERSING) {
    setMotorOutput(BACKWARD_SPEED);
    steeringServo.write(SERVO_CENTER);
    finalServoAngle = SERVO_CENTER;
    if (millis() - lastReverseCmd > 250) {
      // Pi stopped sending REVERSE — clearance regained; resume avoiding (NOT straight:
      // we are still beside a pillar, and the Pi will send CLEAR when it truly is clear).
      currentState = OBSTACLE_AVOIDING;
      lastObstacleCmd = millis();
    }
  }
  else if (currentState == TURNING) {
    executeTurnMode(currentHeading);
  }


  static uint8_t telemetryDiv = 0;   // A12: telemetry every 5th loop — a full line costs ~10 ms of serial time
  if (++telemetryDiv >= 5) {
    telemetryDiv = 0;
    printTelemetry(currentHeading);
  }
  delay(20);
}


