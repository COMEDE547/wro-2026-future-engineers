#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// =====================================================
// Current hardware pins — unchanged
// =====================================================

#define I2C_SDA 21
#define I2C_SCL 22

#define SERVO_PIN 13

#define MOTOR_IN1 25
#define MOTOR_IN2 26
#define MOTOR_PWM 33

#define START_BUTTON 32

// =====================================================
// I2C addresses
// =====================================================

#define PCA9548A_ADDRESS 0x70
#define TFLUNA_ADDRESS   0x10
#define BNO055_ADDRESS   0x28

// =====================================================
// Multiplexer channels — unchanged
// =====================================================

#define MUX_CH_LEFT    0
#define MUX_CH_CENTER  3
#define MUX_CH_RIGHT   2
#define MUX_CH_BNO     4

// =====================================================
// Servo settings
// =====================================================

const int SERVO_CENTER = 106;

const int SERVO_MIN_PULSE_US = 500;
const int SERVO_MAX_PULSE_US = 2400;
const int SERVO_FRAME_MS = 20;

const int FOLLOW_MAX_CORRECTION = 25;
const int TURN_MAX_CORRECTION = 35;

// =====================================================
// Motor speeds — unchanged
// =====================================================

const int DETECTION_SPEED = 190;
const int FOLLOW_SPEED = 190;
const int TURN_SPEED = 160;
const int AFTER_TURN_SPEED = 210;

// =====================================================
// Automatic direction detection
// =====================================================

const int DIRECTION_DETECTION_DISTANCE_CM = 100;
const int DIRECTION_CONFIRMATION_SAMPLES = 3;

// =====================================================
// Wall-following settings
// =====================================================

const float TARGET_LEFT_DISTANCE_CM = 30.0;
const float TARGET_RIGHT_DISTANCE_CM = 30.0;

const int LEFT_OPEN_DISTANCE_CM = 75;
const int RIGHT_OPEN_DISTANCE_CM = 50;

const int LEFT_REACQUIRE_DISTANCE_CM = 50;
const int RIGHT_REACQUIRE_DISTANCE_CM = 50;

const int OPENING_CONFIRMATION_SAMPLES = 3;
const int WALL_CONFIRMATION_SAMPLES = 3;

const unsigned long CLOCKWISE_AFTER_TURN_MS = 500;
const unsigned long ANTICLOCKWISE_AFTER_TURN_MS = 0;

// =====================================================
// Final stopping distance
// =====================================================

const int FINAL_CENTER_DISTANCE_CM = 150;

// =====================================================
// Controller settings
// =====================================================

const float HEADING_KP = 0.55;
const float HEADING_KD = 0.08;

const float WALL_KP = 0.50;
const int MAX_WALL_CORRECTION = 12;

const float TURN_KP = 0.75;
const float TURN_KD = 0.06;

const float TURN_COMPLETE_ERROR = 4.0;

const int TOTAL_TURNS = 12;

const unsigned long CONTROL_PERIOD_US = 10000;

const int INVALID_DISTANCE = 100000;

// =====================================================
// BNO055
// =====================================================

Adafruit_BNO055 bno(
  55,
  BNO055_ADDRESS,
  &Wire
);

// =====================================================
// Direction and robot states
// =====================================================

enum CourseDirection
{
  DIRECTION_UNKNOWN,
  CLOCKWISE,
  ANTICLOCKWISE
};

enum RobotState
{
  WAITING_TO_START,
  DETECTING_DIRECTION,
  FOLLOWING_WALL,
  TURNING,
  WAITING_FOR_WALL,
  WAITING_FOR_CENTER_DISTANCE,
  FINISHED
};

CourseDirection courseDirection = DIRECTION_UNKNOWN;
CourseDirection directionCandidate = DIRECTION_UNKNOWN;

RobotState robotState = WAITING_TO_START;

// =====================================================
// Navigation data
// =====================================================

float targetHeading = 0;

int completedTurns = 0;
int openingSamples = 0;
int wallSamples = 0;
int turnStableSamples = 0;
int directionSamples = 0;

uint8_t consecutiveBnoFailures = 0;

int lastLeftDistance = INVALID_DISTANCE;
int lastCenterDistance = INVALID_DISTANCE;
int lastRightDistance = INVALID_DISTANCE;

unsigned long lastControlUs = 0;
unsigned long afterTurnStartMs = 0;

// =====================================================
// Heading derivative data
// =====================================================

float previousHeadingError = 0;
unsigned long previousHeadingTimeUs = 0;
bool previousHeadingAvailable = false;

// =====================================================
// Shared servo data
// =====================================================

portMUX_TYPE servoMux =
  portMUX_INITIALIZER_UNLOCKED;

volatile uint16_t requestedServoPulseUs = 1500;
volatile int requestedServoAngle = SERVO_CENTER;

// =====================================================
// Servo functions
// =====================================================

uint16_t angleToPulse(int angle)
{
  angle = constrain(angle, 0, 180);

  return (uint16_t)map(
    angle,
    0,
    180,
    SERVO_MIN_PULSE_US,
    SERVO_MAX_PULSE_US
  );
}

void commandServoCorrection(
  int correction,
  int maximumCorrection
)
{
  correction = constrain(
    correction,
    -maximumCorrection,
    maximumCorrection
  );

  int angle = SERVO_CENTER + correction;
  angle = constrain(angle, 0, 180);

  uint16_t pulse = angleToPulse(angle);

  portENTER_CRITICAL(&servoMux);

  requestedServoAngle = angle;
  requestedServoPulseUs = pulse;

  portEXIT_CRITICAL(&servoMux);
}

void centerSteering()
{
  commandServoCorrection(
    0,
    TURN_MAX_CORRECTION
  );
}

// =====================================================
// Dedicated servo task — Core 0
// =====================================================

void servoPulseTask(void *parameter)
{
  TickType_t lastWakeTime =
    xTaskGetTickCount();

  const TickType_t framePeriod =
    pdMS_TO_TICKS(SERVO_FRAME_MS);

  while (true)
  {
    uint16_t pulseWidth;

    portENTER_CRITICAL(&servoMux);
    pulseWidth = requestedServoPulseUs;
    portEXIT_CRITICAL(&servoMux);

    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(SERVO_PIN, LOW);

    vTaskDelayUntil(
      &lastWakeTime,
      framePeriod
    );
  }
}

// =====================================================
// PCA9548A multiplexer
// =====================================================

bool selectMuxChannel(uint8_t channel)
{
  if (channel > 7)
  {
    return false;
  }

  Wire.beginTransmission(PCA9548A_ADDRESS);
  Wire.write(1 << channel);

  return Wire.endTransmission() == 0;
}

// =====================================================
// TF-Luna functions
// =====================================================

int readTFLuna(uint8_t channel)
{
  if (!selectMuxChannel(channel))
  {
    return INVALID_DISTANCE;
  }

  delayMicroseconds(500);

  Wire.beginTransmission(TFLUNA_ADDRESS);
  Wire.write(0x00);

  if (Wire.endTransmission(false) != 0)
  {
    return INVALID_DISTANCE;
  }

  uint8_t received = Wire.requestFrom(
    (uint8_t)TFLUNA_ADDRESS,
    (uint8_t)9
  );

  if (received < 2)
  {
    while (Wire.available())
    {
      Wire.read();
    }

    return INVALID_DISTANCE;
  }

  uint8_t lowByte = Wire.read();
  uint8_t highByte = Wire.read();

  while (Wire.available())
  {
    Wire.read();
  }

  int distance =
    ((int)highByte << 8) | lowByte;

  if (distance > 0 && distance <= 500)
  {
    return distance;
  }

  return INVALID_DISTANCE;
}

int readTFLunaWithRetry(uint8_t channel)
{
  int distance = readTFLuna(channel);

  if (distance == INVALID_DISTANCE)
  {
    distance = readTFLuna(channel);
  }

  return distance;
}

int readLeftDistance()
{
  return readTFLunaWithRetry(MUX_CH_LEFT);
}

int readCenterDistance()
{
  return readTFLunaWithRetry(MUX_CH_CENTER);
}

int readRightDistance()
{
  return readTFLunaWithRetry(MUX_CH_RIGHT);
}

// =====================================================
// BNO055 functions
// =====================================================

float normalizeAngle(float angle)
{
  while (angle > 180)
  {
    angle -= 360;
  }

  while (angle < -180)
  {
    angle += 360;
  }

  return angle;
}

float calculateHeadingError(
  float currentHeading,
  float requiredHeading
)
{
  return normalizeAngle(
    currentHeading - requiredHeading
  );
}

bool readHeading(float &heading)
{
  if (!selectMuxChannel(MUX_CH_BNO))
  {
    return false;
  }

  delayMicroseconds(500);

  sensors_event_t orientationData;

  bool success = bno.getEvent(
    &orientationData,
    Adafruit_BNO055::VECTOR_EULER
  );

  if (!success)
  {
    return false;
  }

  float rawHeading =
    orientationData.orientation.x;

  if (isnan(rawHeading) || isinf(rawHeading))
  {
    return false;
  }

  heading = normalizeAngle(rawHeading);

  return true;
}

// =====================================================
// Heading controller
// =====================================================

void resetHeadingController()
{
  previousHeadingError = 0;
  previousHeadingTimeUs = micros();
  previousHeadingAvailable = false;
}

float headingController(
  float error,
  float kp,
  float kd,
  unsigned long currentTimeUs
)
{
  float errorRate = 0;

  if (previousHeadingAvailable)
  {
    float deltaTime =
      (currentTimeUs - previousHeadingTimeUs)
      / 1000000.0;

    if (deltaTime > 0)
    {
      float errorChange = normalizeAngle(
        error - previousHeadingError
      );

      errorRate = errorChange / deltaTime;
    }
  }

  previousHeadingError = error;
  previousHeadingTimeUs = currentTimeUs;
  previousHeadingAvailable = true;

  return -(kp * error + kd * errorRate);
}

// =====================================================
// Motor control
// =====================================================

void driveForward(int speedValue)
{
  speedValue = constrain(speedValue, 0, 255);

  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);

  analogWrite(MOTOR_PWM, speedValue);
}

void stopMotor()
{
  analogWrite(MOTOR_PWM, 0);

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void brakeMotor()
{
  // Remove motor power before changing bridge state
  analogWrite(MOTOR_PWM, 0);

  // Both L293D outputs at the same level
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  // Enable bridge fully for dynamic braking
  analogWrite(MOTOR_PWM, 255);
}

// =====================================================
// Direction helper functions
// =====================================================

const char *directionName()
{
  switch (courseDirection)
  {
    case CLOCKWISE:
      return "CLOCKWISE";

    case ANTICLOCKWISE:
      return "ANTICLOCKWISE";

    default:
      return "UNKNOWN";
  }
}

int activeWallDistance()
{
  if (courseDirection == CLOCKWISE)
  {
    return lastRightDistance;
  }

  if (courseDirection == ANTICLOCKWISE)
  {
    return lastLeftDistance;
  }

  return INVALID_DISTANCE;
}

int openingDistanceThreshold()
{
  if (courseDirection == CLOCKWISE)
  {
    return RIGHT_OPEN_DISTANCE_CM;
  }

  return LEFT_OPEN_DISTANCE_CM;
}

int reacquireDistanceThreshold()
{
  if (courseDirection == CLOCKWISE)
  {
    return RIGHT_REACQUIRE_DISTANCE_CM;
  }

  return LEFT_REACQUIRE_DISTANCE_CM;
}

unsigned long afterTurnMinimumTime()
{
  if (courseDirection == CLOCKWISE)
  {
    return CLOCKWISE_AFTER_TURN_MS;
  }

  return ANTICLOCKWISE_AFTER_TURN_MS;
}

// =====================================================
// Automatic direction detection
// =====================================================

void updateDirectionDetection()
{
  CourseDirection newCandidate = DIRECTION_UNKNOWN;

  // Preserve original right-side priority
  if (
    lastRightDistance != INVALID_DISTANCE &&
    lastRightDistance >=
      DIRECTION_DETECTION_DISTANCE_CM
  )
  {
    newCandidate = CLOCKWISE;
  }
  else if (
    lastLeftDistance != INVALID_DISTANCE &&
    lastLeftDistance >=
      DIRECTION_DETECTION_DISTANCE_CM
  )
  {
    newCandidate = ANTICLOCKWISE;
  }

  if (newCandidate == DIRECTION_UNKNOWN)
  {
    directionCandidate = DIRECTION_UNKNOWN;
    directionSamples = 0;
    return;
  }

  if (newCandidate == directionCandidate)
  {
    directionSamples++;
  }
  else
  {
    directionCandidate = newCandidate;
    directionSamples = 1;
  }

  if (
    directionSamples >=
    DIRECTION_CONFIRMATION_SAMPLES
  )
  {
    courseDirection = directionCandidate;

    openingSamples = 0;
    wallSamples = 0;

    resetHeadingController();

    driveForward(FOLLOW_SPEED);
    robotState = FOLLOWING_WALL;

    Serial.print("Direction selected: ");
    Serial.println(directionName());
  }
}

// =====================================================
// Begin normal corner turn
// =====================================================

void beginTurn()
{
  if (courseDirection == CLOCKWISE)
  {
    // Right turn: BNO heading increases
    targetHeading = normalizeAngle(
      targetHeading + 90.0
    );
  }
  else
  {
    // Left turn: BNO heading decreases
    targetHeading = normalizeAngle(
      targetHeading - 90.0
    );
  }

  openingSamples = 0;
  turnStableSamples = 0;

  resetHeadingController();

  driveForward(TURN_SPEED);
  robotState = TURNING;

  Serial.print("Starting ");
  Serial.print(directionName());
  Serial.print(" turn. Target: ");
  Serial.println(targetHeading, 1);
}

// =====================================================
// Final electrical brake
// =====================================================

void finishRobot()
{
  brakeMotor();
  centerSteering();

  robotState = FINISHED;

  Serial.print("Center distance reached: ");
  Serial.print(lastCenterDistance);
  Serial.println(" cm");

  Serial.println("Electrical brake applied");
  Serial.println("Brake will remain engaged");
}

// =====================================================
// Final straight approach after turn 12
// =====================================================

void beginFinalApproach()
{
  lastCenterDistance = INVALID_DISTANCE;
  consecutiveBnoFailures = 0;

  centerSteering();
  resetHeadingController();

  // Keep the final heading established by turn 12
  driveForward(FOLLOW_SPEED);

  robotState = WAITING_FOR_CENTER_DISTANCE;

  Serial.println("Twelve turns complete");
  Serial.print("Final straight target heading: ");
  Serial.println(targetHeading, 1);
  Serial.println("Moving toward 150 cm center distance");

  unsigned long finalControlUs = micros();

  // Dedicated final straight-line control loop
  while (
    robotState ==
    WAITING_FOR_CENTER_DISTANCE
  )
  {
    unsigned long currentTimeUs = micros();

    if (
      currentTimeUs - finalControlUs <
      CONTROL_PERIOD_US
    )
    {
      delay(1);
      continue;
    }

    finalControlUs = currentTimeUs;

    // Read BNO055 every final control cycle
    float currentHeading;

    if (!readHeading(currentHeading))
    {
      consecutiveBnoFailures++;

      if (consecutiveBnoFailures >= 10)
      {
        brakeMotor();
        centerSteering();

        robotState = FINISHED;

        Serial.println(
          "SAFETY STOP: BNO failed during final approach"
        );

        Serial.println(
          "Electrical brake applied"
        );

        return;
      }

      continue;
    }

    consecutiveBnoFailures = 0;

    // Read center TF-Luna on channel 3
    lastCenterDistance = readCenterDistance();

    // Brake at or below 150 cm
    if (
      lastCenterDistance != INVALID_DISTANCE &&
      lastCenterDistance <=
        FINAL_CENTER_DISTANCE_CM
    )
    {
      finishRobot();
      return;
    }

    // Maintain the final straight heading
    float headingError = calculateHeadingError(
      currentHeading,
      targetHeading
    );

    float headingCorrection = headingController(
      headingError,
      HEADING_KP,
      HEADING_KD,
      currentTimeUs
    );

    int correction = constrain(
      (int)roundf(headingCorrection),
      -FOLLOW_MAX_CORRECTION,
      FOLLOW_MAX_CORRECTION
    );

    commandServoCorrection(
      correction,
      FOLLOW_MAX_CORRECTION
    );
  }
}

// =====================================================
// Setup
// =====================================================

void setup()
{
  Serial.begin(115200);
  delay(500);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  stopMotor();

  // Initialize motor PWM before servo task starts
  analogWrite(MOTOR_PWM, 0);

  pinMode(START_BUTTON, INPUT_PULLUP);

  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);

  centerSteering();

  BaseType_t servoTaskResult =
    xTaskCreatePinnedToCore(
      servoPulseTask,
      "ServoPulse",
      2048,
      NULL,
      3,
      NULL,
      0
    );

  if (servoTaskResult != pdPASS)
  {
    Serial.println("ERROR: Servo task failed");

    while (true)
    {
      stopMotor();
      delay(100);
    }
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(20);

  if (!selectMuxChannel(MUX_CH_BNO))
  {
    Serial.println("ERROR: PCA9548A not detected");

    while (true)
    {
      stopMotor();
      delay(100);
    }
  }

  delay(10);

  if (!bno.begin())
  {
    Serial.println(
      "ERROR: BNO055 not detected on channel 4"
    );

    while (true)
    {
      stopMotor();
      delay(100);
    }
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("Automatic wall follower ready");
  Serial.println("Press the start button");
}

// =====================================================
// Main loop — Core 1
// =====================================================

void loop()
{
  // Keep the electrical brake latched indefinitely
  if (robotState == FINISHED)
  {
    delay(100);
    return;
  }

  // ---------------------------------------------------
  // Start button
  // ---------------------------------------------------

  if (robotState == WAITING_TO_START)
  {
    if (digitalRead(START_BUTTON) == LOW)
    {
      delay(30);

      if (digitalRead(START_BUTTON) == LOW)
      {
        while (digitalRead(START_BUTTON) == LOW)
        {
          delay(5);
        }

        if (!readHeading(targetHeading))
        {
          Serial.println("Could not read starting heading");
          return;
        }

        courseDirection = DIRECTION_UNKNOWN;
        directionCandidate = DIRECTION_UNKNOWN;

        completedTurns = 0;
        openingSamples = 0;
        wallSamples = 0;
        turnStableSamples = 0;
        directionSamples = 0;

        consecutiveBnoFailures = 0;

        lastLeftDistance = INVALID_DISTANCE;
        lastCenterDistance = INVALID_DISTANCE;
        lastRightDistance = INVALID_DISTANCE;

        resetHeadingController();

        lastControlUs = micros();

        driveForward(DETECTION_SPEED);
        robotState = DETECTING_DIRECTION;

        Serial.println("Detecting course direction");
      }
    }

    delay(2);
    return;
  }

  // ---------------------------------------------------
  // Control timing
  // ---------------------------------------------------

  unsigned long currentTimeUs = micros();

  if (
    currentTimeUs - lastControlUs <
    CONTROL_PERIOD_US
  )
  {
    return;
  }

  lastControlUs = currentTimeUs;

  // ---------------------------------------------------
  // Read BNO055
  // ---------------------------------------------------

  float currentHeading;

  if (!readHeading(currentHeading))
  {
    consecutiveBnoFailures++;

    if (consecutiveBnoFailures >= 10)
    {
      brakeMotor();
      centerSteering();

      robotState = FINISHED;

      Serial.println(
        "SAFETY STOP: Repeated BNO failures"
      );

      Serial.println(
        "Electrical brake applied"
      );
    }

    return;
  }

  consecutiveBnoFailures = 0;

  // ---------------------------------------------------
  // Read required TF-Luna sensors
  // ---------------------------------------------------

  if (robotState == DETECTING_DIRECTION)
  {
    lastLeftDistance = readLeftDistance();
    lastRightDistance = readRightDistance();
  }
  else if (courseDirection == CLOCKWISE)
  {
    lastRightDistance = readRightDistance();
  }
  else if (courseDirection == ANTICLOCKWISE)
  {
    lastLeftDistance = readLeftDistance();
  }

  float headingError = calculateHeadingError(
    currentHeading,
    targetHeading
  );

  int correction = 0;

  // ---------------------------------------------------
  // Detect clockwise or anticlockwise
  // ---------------------------------------------------

  if (robotState == DETECTING_DIRECTION)
  {
    float headingCorrection = headingController(
      headingError,
      HEADING_KP,
      HEADING_KD,
      currentTimeUs
    );

    correction = constrain(
      (int)roundf(headingCorrection),
      -FOLLOW_MAX_CORRECTION,
      FOLLOW_MAX_CORRECTION
    );

    commandServoCorrection(
      correction,
      FOLLOW_MAX_CORRECTION
    );

    updateDirectionDetection();
  }

  // ---------------------------------------------------
  // Follow selected wall
  // ---------------------------------------------------

  else if (robotState == FOLLOWING_WALL)
  {
    int wallDistance = activeWallDistance();

    float headingCorrection = headingController(
      headingError,
      HEADING_KP,
      HEADING_KD,
      currentTimeUs
    );

    float wallCorrection = 0;

    if (wallDistance != INVALID_DISTANCE)
    {
      if (courseDirection == CLOCKWISE)
      {
        float wallError =
          wallDistance - TARGET_RIGHT_DISTANCE_CM;

        wallCorrection = WALL_KP * wallError;
      }
      else
      {
        float wallError =
          TARGET_LEFT_DISTANCE_CM - wallDistance;

        wallCorrection = WALL_KP * wallError;
      }

      wallCorrection = constrain(
        wallCorrection,
        -MAX_WALL_CORRECTION,
        MAX_WALL_CORRECTION
      );
    }

    correction = (int)roundf(
      headingCorrection + wallCorrection
    );

    correction = constrain(
      correction,
      -FOLLOW_MAX_CORRECTION,
      FOLLOW_MAX_CORRECTION
    );

    commandServoCorrection(
      correction,
      FOLLOW_MAX_CORRECTION
    );

    if (
      wallDistance != INVALID_DISTANCE &&
      wallDistance >= openingDistanceThreshold()
    )
    {
      openingSamples++;

      if (
        openingSamples >=
        OPENING_CONFIRMATION_SAMPLES
      )
      {
        beginTurn();
      }
    }
    else
    {
      openingSamples = 0;
    }
  }

  // ---------------------------------------------------
  // Perform selected 90-degree turn
  // ---------------------------------------------------

  else if (robotState == TURNING)
  {
    float turnOutput = headingController(
      headingError,
      TURN_KP,
      TURN_KD,
      currentTimeUs
    );

    correction = constrain(
      (int)roundf(turnOutput),
      -TURN_MAX_CORRECTION,
      TURN_MAX_CORRECTION
    );

    commandServoCorrection(
      correction,
      TURN_MAX_CORRECTION
    );

    if (fabs(headingError) <= TURN_COMPLETE_ERROR)
    {
      turnStableSamples++;

      if (turnStableSamples >= 3)
      {
        completedTurns++;

        if (completedTurns >= TOTAL_TURNS)
        {
          // Continue straight using BNO until
          // center distance reaches 150 cm.
          beginFinalApproach();
          return;
        }

        centerSteering();
        resetHeadingController();

        wallSamples = 0;
        afterTurnStartMs = millis();

        driveForward(AFTER_TURN_SPEED);
        robotState = WAITING_FOR_WALL;

        Serial.print("Turn completed: ");
        Serial.println(completedTurns);
      }
    }
    else
    {
      turnStableSamples = 0;
    }
  }

  // ---------------------------------------------------
  // Reacquire selected wall
  // ---------------------------------------------------

  else if (robotState == WAITING_FOR_WALL)
  {
    int wallDistance = activeWallDistance();

    float headingCorrection = headingController(
      headingError,
      HEADING_KP,
      HEADING_KD,
      currentTimeUs
    );

    correction = constrain(
      (int)roundf(headingCorrection),
      -FOLLOW_MAX_CORRECTION,
      FOLLOW_MAX_CORRECTION
    );

    commandServoCorrection(
      correction,
      FOLLOW_MAX_CORRECTION
    );

    bool minimumTravelComplete =
      millis() - afterTurnStartMs >=
      afterTurnMinimumTime();

    if (
      minimumTravelComplete &&
      wallDistance != INVALID_DISTANCE &&
      wallDistance <
        reacquireDistanceThreshold()
    )
    {
      wallSamples++;

      if (
        wallSamples >=
        WALL_CONFIRMATION_SAMPLES
      )
      {
        wallSamples = 0;
        openingSamples = 0;

        resetHeadingController();

        driveForward(FOLLOW_SPEED);
        robotState = FOLLOWING_WALL;

        Serial.println("Wall reacquired");
      }
    }
    else
    {
      wallSamples = 0;
    }
  }
}

