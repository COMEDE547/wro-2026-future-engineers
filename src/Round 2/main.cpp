#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "BluetoothSerial.h"
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>

#define SERVO_PIN 13

#define MOTOR_IN1  25
#define MOTOR_IN2  26
#define MOTOR_PWM  33

#define MUX_CH_LEFT   0
#define MUX_CH_CENTER 1  
#define MUX_CH_RIGHT  2
#define MUX_CH_BNO    5

#define TFLUNA_I2C_ADDR 0x10

float SLOW_STEER_THRESHOLD = 50.0; // Angle (in degrees) where the servo starts returning to center
int STRAIGHT_SPEED = 100;  // Cruise speed for driving straight (0-255)
int TURN_SPEED     = 100;  // Controlled speed for turning to prevent overshooting
int BACKWARD_SPEED = -100; // Speed for backing up (if needed)


int SERVO_CENTER   = 100;   // Dead-center steering alignment
int DIFF = 35; 
int SERVO_MAX_LEFT = SERVO_CENTER + DIFF;  // Physical mechanical limit for left turn
int SERVO_MAX_RIGHT= SERVO_CENTER - DIFF;   // Physical mechanical limit for right turn

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

const float CARDINAL_HEADINGS[4] = {0.0, 90.0, 180.0, 270.0};

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo steeringServo;
BluetoothSerial SerialBT;    // Added Bluetooth object

enum RobotState { DRIVING_STRAIGHT, TURNING, ROBOT_STOPPED, OBSTACLE_AVOIDING };
RobotState currentState = DRIVING_STRAIGHT;

bool avoidDirectionRight = true;   // true = swerve right (for RED), false = swerve left (for GREEN)
unsigned long lastObstacleCmd = 0; // timeout safety
const unsigned long OBSTACLE_TIMEOUT_MS = 5000; // auto‑clear if no command for 2s

float straightTargetHeading = 0.0;
float turnTargetHeading     = 0.0;
bool isTurningLeft          = false;
int totalTurnsCount         = 0;
bool hasTurnedOnce          = false; // Becomes true permanently on the first turn
bool lockedDirectionLeft    = false; // Remembers if our layout is strictly Left or Right

int16_t currentLeftDist     = -1;
int16_t currentCenterDist   = -1; 
int16_t currentRightDist    = -1;
int finalServoAngle         = 90;
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



float snapToCardinal(float angle) {
  angle = fmod(angle, 360.0);
  if (angle < 0.0) angle += 360.0;

  float best = CARDINAL_HEADINGS[0];
  float bestDiff = 999.0;

  for (int i = 0; i < 4; i++) {
    float diff = fabs(angle - CARDINAL_HEADINGS[i]);
    if (diff > 180.0) diff = 360.0 - diff; // shortest wrap-around distance
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
  if (millis() < turnCooldownUntil) return; 
  if (currentLeftDist == -1 || currentRightDist == -1) return;


  int16_t difference = currentLeftDist - currentRightDist;

  if (hasTurnedOnce) {
    if (lockedDirectionLeft) {
      if (difference > SPIKE_THRESHOLD) {
        float currentHeading = getCurrentHeading();
        isTurningLeft = true;
        turnTargetHeading = computeTurnTarget(currentHeading, true);
        currentState = TURNING;
      }
    } else {
      if (difference < -SPIKE_THRESHOLD) {
        float currentHeading = getCurrentHeading();
        isTurningLeft = false;
        turnTargetHeading = computeTurnTarget(currentHeading, false);
        currentState = TURNING;
      }
    }
    return;
  }


  // 2. FIRST TURN RUN
  if (difference > SPIKE_THRESHOLD) {
    float currentHeading = getCurrentHeading();
    isTurningLeft = true;
    turnTargetHeading = computeTurnTarget(currentHeading, true);
    hasTurnedOnce = true;
    lockedDirectionLeft = true;
   
    Serial.println("!!! LAYOUT INITIALIZED: PERMANENT LEFT TURN ONLY LOCK ACTIVATED !!!");
    SerialBT.println("!!! LAYOUT INITIALIZED: PERMANENT LEFT TURN ONLY LOCK ACTIVATED !!!");


    currentState = TURNING;
  }
  else if (difference < -SPIKE_THRESHOLD) {
    float currentHeading = getCurrentHeading();
    isTurningLeft = false;
    turnTargetHeading = computeTurnTarget(currentHeading, false);
    hasTurnedOnce = true;
    lockedDirectionLeft = false;
   
    Serial.println("!!! LAYOUT INITIALIZED: PERMANENT RIGHT TURN ONLY LOCK ACTIVATED !!!");
    SerialBT.println("!!! LAYOUT INITIALIZED: PERMANENT RIGHT TURN ONLY LOCK ACTIVATED !!!");


    currentState = TURNING;
  }
}

void avoidObstacle() {

  setMotorOutput(STRAIGHT_SPEED);  // or a slightly slower speed

  // Determine swerve direction
  int swerveAngle;
  if (avoidDirectionRight) {
    // Swerve right – depending on INVERT_STEERING
    swerveAngle = INVERT_STEERING ? SERVO_MAX_LEFT : SERVO_MAX_RIGHT;
  } else {
    swerveAngle = INVERT_STEERING ? SERVO_MAX_RIGHT : SERVO_MAX_LEFT;
  }


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


template <typename T>
void btPrint(T msg) {
  Serial.print(msg);
  SerialBT.print(msg);
}


template <typename T>
void btPrintln(T msg) {
  Serial.println(msg);
  SerialBT.println(msg);
}


void printTelemetry(float currentHeading) {
  if (currentState == DRIVING_STRAIGHT) btPrint("MODE: STRAIGHT");
  else if (currentState == TURNING)     btPrint("MODE: TURNING ");


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
  SerialBT.begin("ESP32_Robot_Telemetry");

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);


  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);


  selectMuxChannel(MUX_CH_BNO);
  if (!bno.begin()) {
    Serial.println("Critical Error: BNO055 missing on Multiplexer channel 4!");
    SerialBT.println("Critical Error: BNO055 missing on Multiplexer channel 4!");
      }
      else {
    Serial.println("WARNING: No BNO055 - running without IMU");
    SerialBT.println("WARNING: No BNO055 - running without IMU");
    // NO while(1) — just continue
  }
  // delay(500);
  // bno.setExtCrystalUse(true);


  straightTargetHeading = getCurrentHeading();
  Serial.println("=== ROBOT READY ===");
  Serial.println("Commands: RED, GREEN, CLEAR");
  SerialBT.println("=== ROBOT READY ===");
}


void loop() {

static String serialBuffer = "";
while (Serial.available()) {
  char c = Serial.read();
  if (c == '\n') {

    serialBuffer.trim();
    if (serialBuffer == "RED") {
      if (currentState != OBSTACLE_AVOIDING) {

        avoidDirectionRight = true; 
        currentState = OBSTACLE_AVOIDING;
        Serial.println("OBSTACLE: RED - Swerving RIGHT");
      }
      lastObstacleCmd = millis();
    } 
    else if (serialBuffer == "GREEN") {
      if (currentState != OBSTACLE_AVOIDING) {
        avoidDirectionRight = false;  
        currentState = OBSTACLE_AVOIDING;
        Serial.println("OBSTACLE: GREEN - Swerving LEFT");
      }
      lastObstacleCmd = millis();
    } 
    else if (serialBuffer == "CLEAR") {
      if (currentState == OBSTACLE_AVOIDING) {
        currentState = DRIVING_STRAIGHT;

        lastHeadingError = 0;
        lastHeadingTime = millis();
        Serial.println("OBSTACLE: CLEARED - Returning to path");
      }
    }
    serialBuffer = "";
  } else {
    serialBuffer += c;
  }
}

if (currentState == OBSTACLE_AVOIDING && (millis() - lastObstacleCmd > OBSTACLE_TIMEOUT_MS)) {
  currentState = DRIVING_STRAIGHT;
  lastHeadingError = 0;
  lastHeadingTime = millis();
  Serial.println("OBSTACLE: Timeout - Auto returning to path");
}


  if (currentState == ROBOT_STOPPED) {
    setMotorOutput(0);
    steeringServo.write(SERVO_CENTER);
   
    Serial.print("STATUS: Finished. Total Turns Executed: ");
    Serial.println(totalTurnsCount);
    SerialBT.print("STATUS: Finished. Total Turns Executed: ");
    SerialBT.println(totalTurnsCount);
   
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
  else if (currentState == TURNING) {
    executeTurnMode(currentHeading);
  }


  printTelemetry(currentHeading);
  delay(20);
}


