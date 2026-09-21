#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_NeoPixel.h>
#include <utility/imumaths.h>
#include <math.h>
#include <errno.h>
#include <limits.h>


// =====================================================
// Current hardware pins
// =====================================================
#define LED_PIN 4
#define LED_COUNT 6

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define I2C_SDA 21
#define I2C_SCL 22

#define SERVO_PIN 13

#define MOTOR_IN1 25
#define MOTOR_IN2 26
#define MOTOR_PWM 33

#define START_BUTTON 32

// =====================================================
// I2C addresses and PCA9548A channels
// =====================================================

#define PCA9548A_ADDRESS 0x70
#define TFLUNA_ADDRESS   0x10
#define BNO055_ADDRESS   0x28

#define MUX_CH_LEFT    0
#define MUX_CH_CENTER  3
#define MUX_CH_RIGHT   2
#define MUX_CH_BNO     4

// =====================================================
// Servo configuration
// =====================================================

const int SERVO_CENTER = 85;
const int SERVO_MIN_PULSE_US = 500;
const int SERVO_MAX_PULSE_US = 2400;
const int SERVO_FRAME_MS = 20;
const int SERVO_MIN_CORRECTION = -60;
const int SERVO_MAX_CORRECTION = 60;

portMUX_TYPE servoMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint16_t requestedServoPulseUs = 1500;
volatile int requestedServoCorrection = 0;

// =====================================================
// Sensors and serial protocol
// =====================================================

Adafruit_BNO055 bno(55, BNO055_ADDRESS, &Wire);

const int INVALID_DISTANCE = -1;
const int INVALID_HEADING = 1000;

const unsigned long DISTANCE_FRESH_MS = 250;
const unsigned long HEADING_FRESH_MS = 250;

const uint8_t VALID_LEFT_MASK = 0x01;
const uint8_t VALID_CENTER_MASK = 0x02;
const uint8_t VALID_RIGHT_MASK = 0x04;
const uint8_t VALID_HEADING_MASK = 0x08;

int distanceLeft = INVALID_DISTANCE;
int distanceCenter = INVALID_DISTANCE;
int distanceRight = INVALID_DISTANCE;

uint8_t leftReadFailures = 0;
uint8_t centerReadFailures = 0;
uint8_t rightReadFailures = 0;

unsigned long leftLastValidMillis = 0;
unsigned long centerLastValidMillis = 0;
unsigned long rightLastValidMillis = 0;

bool leftHasValidReading = false;
bool centerHasValidReading = false;
bool rightHasValidReading = false;

int heading = 0;
uint8_t headingReadFailures = 0;
unsigned long headingLastValidMillis = 0;
bool headingHasValidReading = false;

String commandInput = "";

const unsigned long COMMAND_TIMEOUT_MS = 500;
const int HARD_STOP_ENTER_CM = 18;
const int HARD_STOP_RELEASE_CM = 24;
const unsigned long PARKING_OVERRIDE_TIMEOUT_MS = 10000;
const unsigned int MOTOR_DIRECTION_DEADTIME_US = 2000;

unsigned long lastCommandMillis = 0;
bool commandReceived = false;

bool collisionHardStopLatched = false;
bool reverseReleaseObserved = false;
bool parkingDistanceOverride = false;
unsigned long parkingOverrideLastCommandMillis = 0;

int appliedSpeed = 0;
int appliedDirection = 0;
bool motorBrakeEngaged = false;
int lastNonzeroDirection = 0;
bool hasDriven = false;

uint32_t telemetrySequence = 0;

// =====================================================
// Servo pulse generation on Core 0
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

void steer(int correction)
{
  correction = constrain(
    correction,
    SERVO_MIN_CORRECTION,
    SERVO_MAX_CORRECTION
  );

  int angle = SERVO_CENTER + correction;
  angle = constrain(angle, 0, 180);

  uint16_t pulse = angleToPulse(angle);

  portENTER_CRITICAL(&servoMux);
  requestedServoCorrection = correction;
  requestedServoPulseUs = pulse;
  portEXIT_CRITICAL(&servoMux);
}

void servoPulseTask(void *parameter)
{
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t framePeriod = pdMS_TO_TICKS(SERVO_FRAME_MS);

  while (true)
  {
    uint16_t pulseWidth;

    portENTER_CRITICAL(&servoMux);
    pulseWidth = requestedServoPulseUs;
    portEXIT_CRITICAL(&servoMux);

    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(SERVO_PIN, LOW);

    vTaskDelayUntil(&lastWakeTime, framePeriod);
  }
}

// =====================================================
// PCA9548A and TF-Luna functions
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

  int distance = ((int)highByte << 8) | lowByte;

  if (distance > 0 && distance <= 500)
  {
    return distance;
  }

  return INVALID_DISTANCE;
}

void updateDistance(
  uint8_t channel,
  int &distanceValue,
  uint8_t &failureCount,
  unsigned long &lastValidMillis,
  bool &hasValidReading
)
{
  int reading = readTFLuna(channel);

  if (reading == INVALID_DISTANCE)
  {
    reading = readTFLuna(channel);
  }

  if (reading != INVALID_DISTANCE)
  {
    distanceValue = reading;
    failureCount = 0;
    lastValidMillis = millis();
    hasValidReading = true;
    return;
  }

  if (failureCount < 255)
  {
    failureCount++;
  }
}

void updateTFLunas()
{
  updateDistance(
    MUX_CH_LEFT,
    distanceLeft,
    leftReadFailures,
    leftLastValidMillis,
    leftHasValidReading
  );

  updateDistance(
    MUX_CH_CENTER,
    distanceCenter,
    centerReadFailures,
    centerLastValidMillis,
    centerHasValidReading
  );

  updateDistance(
    MUX_CH_RIGHT,
    distanceRight,
    rightReadFailures,
    rightLastValidMillis,
    rightHasValidReading
  );
}

bool readingIsFresh(
  bool hasValidReading,
  unsigned long lastValidMillis,
  unsigned long freshnessLimit,
  unsigned long now
)
{
  return hasValidReading &&
         (unsigned long)(now - lastValidMillis) <= freshnessLimit;
}

bool leftIsFresh(unsigned long now)
{
  return readingIsFresh(
    leftHasValidReading,
    leftLastValidMillis,
    DISTANCE_FRESH_MS,
    now
  );
}

bool centerIsFresh(unsigned long now)
{
  return readingIsFresh(
    centerHasValidReading,
    centerLastValidMillis,
    DISTANCE_FRESH_MS,
    now
  );
}

bool rightIsFresh(unsigned long now)
{
  return readingIsFresh(
    rightHasValidReading,
    rightLastValidMillis,
    DISTANCE_FRESH_MS,
    now
  );
}

// =====================================================
// BNO055
// =====================================================

void updateBNO()
{
  if (!selectMuxChannel(MUX_CH_BNO))
  {
    if (headingReadFailures < 255)
    {
      headingReadFailures++;
    }

    return;
  }

  delayMicroseconds(500);

  sensors_event_t orientationData;

  if (!bno.getEvent(
    &orientationData,
    Adafruit_BNO055::VECTOR_EULER
  ))
  {
    if (headingReadFailures < 255)
    {
      headingReadFailures++;
    }

    return;
  }

  float rawHeading = orientationData.orientation.x;

  if (isnan(rawHeading) || isinf(rawHeading))
  {
    if (headingReadFailures < 255)
    {
      headingReadFailures++;
    }

    return;
  }

  heading = (int)rawHeading;

  if (heading > 180)
  {
    heading -= 360;
  }

  if (heading < -180)
  {
    heading += 360;
  }

  headingReadFailures = 0;
  headingLastValidMillis = millis();
  headingHasValidReading = true;
}

bool headingIsFresh(unsigned long now)
{
  return readingIsFresh(
    headingHasValidReading,
    headingLastValidMillis,
    HEADING_FRESH_MS,
    now
  );
}

// =====================================================
// Motor control
// =====================================================

void activeBrakeMotor()
{
  if (motorBrakeEngaged && appliedSpeed == 0)
  {
    return;
  }

  // L293D active brake: both bridge inputs equal while ENA is HIGH.
  analogWrite(MOTOR_PWM, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  delayMicroseconds(100);
  analogWrite(MOTOR_PWM, 255);

  appliedSpeed = 0;
  motorBrakeEngaged = true;
}

void driveMotor(int speedValue, int direction)
{
  speedValue = constrain(speedValue, 0, 255);

  if (speedValue == 0)
  {
    activeBrakeMotor();
    return;
  }

  bool directionChanged =
    hasDriven && lastNonzeroDirection != direction;

  if (motorBrakeEngaged || directionChanged)
  {
    // Disable the bridge before changing either direction input.
    analogWrite(MOTOR_PWM, 0);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);

    if (directionChanged)
    {
      delayMicroseconds(MOTOR_DIRECTION_DEADTIME_US);
    }
    else
    {
      delayMicroseconds(100);
    }
  }

  if (direction == 0)
  {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  }
  else
  {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
  }

  analogWrite(MOTOR_PWM, speedValue);

  appliedSpeed = speedValue;
  appliedDirection = direction;
  motorBrakeEngaged = false;
  lastNonzeroDirection = direction;
  hasDriven = true;
}

void stopMotor()
{
  activeBrakeMotor();
}

void updateHardStopLatch(unsigned long now)
{
  bool centerFresh = centerIsFresh(now);

  if (
    centerFresh &&
    distanceCenter <= HARD_STOP_ENTER_CM &&
    !collisionHardStopLatched
  )
  {
    collisionHardStopLatched = true;
    reverseReleaseObserved = false;
  }

  if (
    collisionHardStopLatched &&
    reverseReleaseObserved &&
    centerFresh &&
    distanceCenter >= HARD_STOP_RELEASE_CM
  )
  {
    collisionHardStopLatched = false;
    reverseReleaseObserved = false;
  }
}

bool forwardHardStopIsActive(unsigned long now)
{
  // Disabled: stopping the drive motor here ends a competition run.
  // Distance telemetry remains available to Python for navigation decisions.
  return false;
}

void enforceSafetyInterlocks(unsigned long now)
{
  if (
    parkingDistanceOverride &&
    (unsigned long)(now - parkingOverrideLastCommandMillis) >
      PARKING_OVERRIDE_TIMEOUT_MS
  )
  {
    parkingDistanceOverride = false;
  }

  updateHardStopLatch(now);

  if (appliedSpeed > 0)
  {
    bool staleHeading = !headingIsFresh(now);
    bool unsafeForward =
      appliedDirection == 0 && forwardHardStopIsActive(now);

    if (staleHeading || unsafeForward)
    {
      stopMotor();
    }
  }

  if (
    commandReceived &&
    (unsigned long)(now - lastCommandMillis) > COMMAND_TIMEOUT_MS
  )
  {
    stopMotor();
    steer(0);
    commandReceived = false;
  }
}

// =====================================================
// Raspberry Pi handshake
// =====================================================

void waitForOK()
{
  String handshakeInput = "";
  unsigned long lastOkMessage = 0;

  while (true)
  {
    if (millis() - lastOkMessage >= 500)
    {
      Serial.println("OK");
      lastOkMessage = millis();
    }

    while (Serial.available())
    {
      char c = Serial.read();

      if (c == '\n')
      {
        handshakeInput.trim();

        if (handshakeInput.equalsIgnoreCase("OK"))
        {
          return;
        }

        handshakeInput = "";
      }
      else if (c != '\r')
      {
        handshakeInput += c;
      }
    }

    delay(1);
  }
}

// =====================================================
// Raspberry Pi telemetry and commands
// =====================================================

void sendSensorData()
{
  unsigned long now = millis();
  uint8_t validMask = 0;

  bool leftFresh = leftIsFresh(now);
  bool centerFresh = centerIsFresh(now);
  bool rightFresh = rightIsFresh(now);
  bool headingFresh = headingIsFresh(now);

  if (leftFresh)
  {
    validMask |= VALID_LEFT_MASK;
  }

  if (centerFresh)
  {
    validMask |= VALID_CENTER_MASK;
  }

  if (rightFresh)
  {
    validMask |= VALID_RIGHT_MASK;
  }

  if (headingFresh)
  {
    validMask |= VALID_HEADING_MASK;
  }

  int reportedHeading = headingFresh ? heading : INVALID_HEADING;
  int reportedLeft = leftFresh ? distanceLeft : INVALID_DISTANCE;
  int reportedCenter = centerFresh ? distanceCenter : INVALID_DISTANCE;
  int reportedRight = rightFresh ? distanceRight : INVALID_DISTANCE;
  int hardStop = forwardHardStopIsActive(now) ? 1 : 0;
  int signedAppliedSpeed =
    appliedDirection == 0 ? appliedSpeed : -appliedSpeed;

  telemetrySequence++;

  // Protocol:
  // T,seq,ms,heading,left,center,right,validMask,hardStop,appliedSpeed
  Serial.print("T,");
  Serial.print(telemetrySequence);
  Serial.print(",");
  Serial.print(now);
  Serial.print(",");
  Serial.print(reportedHeading);
  Serial.print(",");
  Serial.print(reportedLeft);
  Serial.print(",");
  Serial.print(reportedCenter);
  Serial.print(",");
  Serial.print(reportedRight);
  Serial.print(",");
  Serial.print(validMask);
  Serial.print(",");
  Serial.print(hardStop);
  Serial.print(",");
  Serial.println(signedAppliedSpeed);
}

bool parseIntegerToken(const char *token, int &value)
{
  if (token == NULL || token[0] == '\0')
  {
    return false;
  }

  errno = 0;
  char *endPointer = NULL;
  long parsed = strtol(token, &endPointer, 10);

  if (
    errno == ERANGE ||
    endPointer == token ||
    *endPointer != '\0' ||
    parsed < INT_MIN ||
    parsed > INT_MAX
  )
  {
    return false;
  }

  value = (int)parsed;
  return true;
}

void processCommand(String command)
{
  command.trim();

  if (command.length() == 0)
  {
    return;
  }

  unsigned long now = millis();

  if (command.equalsIgnoreCase("PARKING_OVERRIDE_ON"))
  {
    parkingDistanceOverride = true;
    parkingOverrideLastCommandMillis = now;
    commandReceived = true;
    lastCommandMillis = now;
    stopMotor();
    return;
  }

  if (command.equalsIgnoreCase("PARKING_OVERRIDE_OFF"))
  {
    parkingDistanceOverride = false;
    commandReceived = true;
    lastCommandMillis = now;
    updateHardStopLatch(now);
    stopMotor();
    return;
  }

  // Protocol: speed,direction,steering
  int parts[3] = {0, 0, 0};
  char buffer[100];

  command.toCharArray(buffer, sizeof(buffer));

  char *savePointer = NULL;
  char *token = strtok_r(buffer, ",", &savePointer);

  for (int index = 0; index < 3; index++)
  {
    if (!parseIntegerToken(token, parts[index]))
    {
      return;
    }

    token = strtok_r(NULL, ",", &savePointer);
  }

  // Reject commands with missing or extra fields.
  if (token != NULL)
  {
    return;
  }

  int speedValue = parts[0];
  int direction = parts[1];
  int steeringCorrection = parts[2];

  if (
    speedValue < 0 ||
    speedValue > 255 ||
    (direction != 0 && direction != 1) ||
    steeringCorrection < SERVO_MIN_CORRECTION ||
    steeringCorrection > SERVO_MAX_CORRECTION
  )
  {
    return;
  }

  commandReceived = true;
  lastCommandMillis = now;
  if (parkingDistanceOverride)
  {
    parkingOverrideLastCommandMillis = now;
  }

  updateHardStopLatch(now);
  steer(steeringCorrection);

  if (speedValue == 0)
  {
    stopMotor();
    return;
  }

  if (!headingIsFresh(now))
  {
    stopMotor();
    steer(0);
    return;
  }

  if (direction == 0 && forwardHardStopIsActive(now))
  {
    stopMotor();
    return;
  }

  if (direction == 1 && collisionHardStopLatched)
  {
    reverseReleaseObserved = true;
  }

  driveMotor(speedValue, direction);
}

void receiveCommands()
{
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n')
    {
      commandInput.trim();
      processCommand(commandInput);
      commandInput = "";
    }
    else if (c != '\r')
    {
      if (commandInput.length() < 99)
      {
        commandInput += c;
      }
      else
      {
        commandInput = "";
      }
    }
  }
}

// =====================================================
// Setup
// =====================================================

void setup()
{
  strip.begin();
  strip.setBrightness(50);

  // Turn ON all 6 LEDs - White
  for (int i = 0; i < LED_COUNT; i++)
  {
    strip.setPixelColor(i, strip.Color(255, 255, 255));
  }

  strip.show();

  Serial.begin(115200);
  delay(500);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  stopMotor();

  pinMode(START_BUTTON, INPUT_PULLUP);

  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);

  steer(0);

  BaseType_t servoTaskResult = xTaskCreatePinnedToCore(
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
    Serial.println("ERROR: SERVO TASK");

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
    Serial.println("ERROR: PCA9548A BNO CHANNEL");

    while (true)
    {
      stopMotor();
      delay(100);
    }
  }

  delay(10);

  if (!bno.begin())
  {
    Serial.println("ERROR: BNO055 NOT FOUND");

    while (true)
    {
      stopMotor();
      delay(100);
    }
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("DEBUG: INITIALIZED");

  // Hold the start button for two seconds.
  bool waitingForButton = true;

  while (waitingForButton)
  {
    if (digitalRead(START_BUTTON) == LOW)
    {
      unsigned long pressStart = millis();

      while (digitalRead(START_BUTTON) == LOW)
      {
        if (millis() - pressStart >= 2000)
        {
          waitingForButton = false;
          break;
        }

        delay(1);
      }
    }

    delay(1);
  }

  Serial.println("Waiting for ok");
  waitForOK();
  Serial.println("Handshake Complete");

  lastCommandMillis = millis();
}

// =====================================================
// Main loop — Core 1
// =====================================================

void loop()
{
  // Process commands before sensor polling.
  receiveCommands();

  updateTFLunas();
  updateBNO();

  enforceSafetyInterlocks(millis());

  // Versioned telemetry includes sensor health and applied motor state.
  sendSensorData();

  // Process commands received while sensors were being read.
  receiveCommands();

  enforceSafetyInterlocks(millis());

  delay(20);
}
