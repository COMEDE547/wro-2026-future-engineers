/* ============================================================================
   bench_teleop_park.ino  —  BENCH TOOL. NOT A COMPETITION BUILD.

   Manual Bluetooth teleop for characterising the parallel-park manoeuvre by
   hand: drive the car into the bay yourself, mark each phase boundary, and
   read out the yaw delta and Luna distances that the park controller needs as
   constants.

   !! WRO rule 11.10 forbids any active radio during a round. This sketch is
   !! Bluetooth-first by design, so it is a bench-only artifact. Do not flash it
   !! at the venue, do not leave it resident, and keep it out of src/.

   Pins, steering limits, I2C addresses and the Luna/BNO read paths are copied
   verbatim from src/Round 2/main.cpp @ 18d7edb so measurements transfer to the
   flight build without a units or calibration gap.

   PAIR:  Bluetooth device "TED_PARK_BENCH"  (Android: Serial Bluetooth Terminal)
   Commands also work over USB serial at 115200 — same single-character keys.

   SAFETY, read once:
     - Reverse is UNGUARDED. There is no rear sensor on this vehicle. Keep ~40 cm
       clear behind the car before any reverse or arc pulse.
     - Forward motion is guarded by the centre Luna at FRONT_GUARD_CM (toggle 'g').
     - Drive latches. It auto-stops after IDLE_STOP_MS with no command, and on
       Bluetooth disconnect. Any keypress refreshes the timer.
     - During a timed pulse, ANY incoming byte aborts the pulse immediately.
   ============================================================================ */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>
#include "BluetoothSerial.h"

/* ---------- pins: identical to src/Round 2/main.cpp ---------- */
#define SERVO_PIN         13
#define MOTOR_IN1         25
#define MOTOR_IN2         26
#define MOTOR_PWM         33
#define MOTOR_STBY        27
#define START_BUTTON_PIN  32

#define MUX_ADDR          0x70
#define MUX_CH_LEFT       0
#define MUX_CH_CENTER     1
#define MUX_CH_RIGHT      2
#define MUX_CH_BNO        4
#define TFLUNA_I2C_ADDR   0x10

/* ---------- steering geometry: main.cpp values, mat-tuned 2026-08-10 --------
   SERVO_MAX_* are servo-angle endpoints, NOT physical directions. invertSteering
   maps driver intent onto an endpoint, exactly as INVERT_STEERING does in the
   flight code. Both invert flags are runtime-togglable here on purpose — this
   sketch is how you settle their true values without a reflash.            */
int  SERVO_CENTER    = 106;   // dead-centre alignment
int  SERVO_MAX_LEFT  = 136;   // higher-angle endpoint (+30 from centre)
int  SERVO_MAX_RIGHT = 64;    // lower-angle endpoint  (-42 from centre)
bool invertSteering  = true;  // 'k' toggles — matches INVERT_STEERING = true
bool invertDrive     = false; // 'i' toggles — set if 'w' drives backwards

/* ---------- tunables ---------- */
const int  STEER_STEP        = 2;     // degrees per a/d press
int        duty              = 90;    // 0..255, current drive PWM
const int  DUTY_STEP         = 10;
const int  DUTY_MIN_USEFUL   = 60;    // below this the N20 through 5:7 may stall

const unsigned long IDLE_STOP_MS = 3000;  // deadman: stop if no command
const unsigned long TEL_PERIOD_MS = 100;  // 10 Hz telemetry
const unsigned long PULSE_MS      = 1000; // 'c'/'v' straight calibration pulse
const unsigned long ARC_PULSE_MS  = 1500; // 'o'/'p' reverse arc pulse
int        FRONT_GUARD_CM    = 15;
bool       frontGuard        = true;
bool       telOn             = false;

/* ---------- state ---------- */
/* Adafruit_BNO055 <=1.5.3 declares its opmode enum INSIDE the class; >=1.6.0
   moved it to global scope, so each version rejects the other's spelling.
   Unqualified lookup inside a class derived from Adafruit_BNO055 resolves the
   name in EITHER location, so this shim compiles against any library version.
   The flight build (src/Round 2/main.cpp) still uses the class-qualified
   spelling and therefore requires <=1.5.3 - that pin matters for the Aug 22
   competition flash. */
struct BnoCompat : Adafruit_BNO055 {
  static auto imuplus() -> decltype(OPERATION_MODE_IMUPLUS) { return OPERATION_MODE_IMUPLUS; }
};

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo         steeringServo;
BluetoothSerial SerialBT;

int           servoAngle  = 106;
int           driveDir    = 0;      // -1 reverse, 0 stop, +1 forward (driver frame)
unsigned long lastCmdMs   = 0;
unsigned long lastTelMs   = 0;
float         yawRef      = 0.0;
int           markN       = 0;
bool          btWasClient = false;

/* ============================ plumbing ==================================== */

void out(const String &s)   { Serial.print(s);   if (SerialBT.hasClient()) SerialBT.print(s); }
void outln(const String &s) { Serial.println(s); if (SerialBT.hasClient()) SerialBT.println(s); }

void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MUX_ADDR);
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
    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    return (int16_t)(lo + (hi << 8));
  }
  return -1;
}

bool lunaValid(int16_t d) { return d >= 2 && d <= 600; }

float getYaw() {
  selectMuxChannel(MUX_CH_BNO);
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

// signed shortest-arc delta from the marked reference, -180..+180
float yawDelta() {
  float d = getYaw() - yawRef;
  if (d >  180.0) d -= 360.0;
  if (d < -180.0) d += 360.0;
  return d;
}

/* ============================ actuators =================================== */

void setMotorOutput(int speed) {          // identical to main.cpp
  if (speed >= 0) { digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW); }
  else            { digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, HIGH); speed = -speed; }
  analogWrite(MOTOR_PWM, constrain(speed, 0, 255));
}

void steerTo(int angle) {
  int lo = min(SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
  int hi = max(SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
  servoAngle = constrain(angle, lo, hi);
  steeringServo.write(servoAngle);
}

// dir +1 = steer RIGHT as the driver sees it. Mirrors main.cpp:309-313.
int endpointFor(int dir) {
  if (dir > 0) return invertSteering ? SERVO_MAX_LEFT  : SERVO_MAX_RIGHT;
  else         return invertSteering ? SERVO_MAX_RIGHT : SERVO_MAX_LEFT;
}

void steerStep(int dir) {
  int ep = endpointFor(dir);
  steerTo(servoAngle + ((ep > SERVO_CENTER) ? STEER_STEP : -STEER_STEP));
}

void setDrive(int dir) {                  // dir in driver frame
  driveDir = dir;
  int signedDuty = dir * duty * (invertDrive ? -1 : 1);
  setMotorOutput(signedDuty);
}

void stopDrive(const char *why) {
  driveDir = 0;
  setMotorOutput(0);
  if (why[0]) { out("[stop] "); outln(why); }
}

/* ============================ reporting =================================== */

String telLine(const char *tag) {
  int16_t L = getLunaDistance(MUX_CH_LEFT);
  int16_t C = getLunaDistance(MUX_CH_CENTER);
  int16_t R = getLunaDistance(MUX_CH_RIGHT);
  String s = String(tag) + " t=" + String(millis());
  s += " yaw=" + String(getYaw(), 1) + " dyaw=" + String(yawDelta(), 1);
  s += " L=" + String(L) + " C=" + String(C) + " R=" + String(R);
  s += " srv=" + String(servoAngle) + " dir=" + String(driveDir) + " duty=" + String(duty);
  s += " btn=" + String(digitalRead(START_BUTTON_PIN));
  return s;
}

void mark() {
  markN++;
  outln(telLine(("[mark] #" + String(markN)).c_str()));
}

void report() {
  outln("---- bench_teleop_park config ----");
  outln("servo   centre=" + String(SERVO_CENTER) +
        " endpoints=" + String(SERVO_MAX_RIGHT) + ".." + String(SERVO_MAX_LEFT) +
        " now=" + String(servoAngle));
  outln("flags   invertSteering=" + String(invertSteering ? 1 : 0) +
        " invertDrive=" + String(invertDrive ? 1 : 0) +
        " frontGuard=" + String(frontGuard ? 1 : 0) + "@" + String(FRONT_GUARD_CM) + "cm");
  outln("drive   duty=" + String(duty) + " dir=" + String(driveDir) +
        " idleStop=" + String(IDLE_STOP_MS) + "ms");
  outln("pulses  straight=" + String(PULSE_MS) + "ms arc=" + String(ARC_PULSE_MS) + "ms");
  outln(telLine("[now]"));
  outln("start button reads " + String(digitalRead(START_BUTTON_PIN)) +
        "  (unpressed should be 1 for button-to-GND + pullup)");
}

void help() {
  outln("---- keys ----");
  outln(" w/s   forward / reverse (latched)      x or space  stop");
  outln(" a/d   steer left / right by 2 deg      f  centre (106)");
  outln(" q/e   full lock left / right");
  outln(" [ ]   duty -/+ 10                      1/2/3  duty 70 / 90 / 120");
  outln(" z     zero yaw reference (mark start)  m  timestamped mark line");
  outln(" t     10 Hz telemetry on/off           r  report config   h  help");
  outln(" c/v   timed 1.0 s pulse fwd / rev  -> measure cm with a tape = cm/s");
  outln(" o/p   1.5 s REVERSE arc at full left / right lock -> prints d_yaw");
  outln(" g     front guard on/off   i  invert drive   k  invert steering");
  outln(" REVERSE IS UNGUARDED - no rear sensor. Clear ~40 cm behind.");
}

/* ============================ timed pulses ================================
   Bounded, blocking, and abortable: any incoming byte on either link stops the
   pulse. Used to convert duty into cm/s ('c'/'v') and to measure the actual
   reverse turn radius ('o'/'p') without guessing wheelbase or steer angle. */

bool pulseAborted() {
  if (Serial.available() || (SerialBT.hasClient() && SerialBT.available())) {
    while (Serial.available()) Serial.read();
    while (SerialBT.hasClient() && SerialBT.available()) SerialBT.read();
    return true;
  }
  return false;
}

void timedPulse(int dir, unsigned long ms, const char *label) {
  float y0 = getYaw();
  outln(String("[pulse] ") + label + " dir=" + String(dir) + " duty=" + String(duty) +
        " srv=" + String(servoAngle) + " ms=" + String(ms) + " yaw0=" + String(y0, 1));
  unsigned long t0 = millis();
  setDrive(dir);
  bool aborted = false;
  while (millis() - t0 < ms) {
    if (pulseAborted()) { aborted = true; break; }
    if (dir > 0 && frontGuard) {
      int16_t c = getLunaDistance(MUX_CH_CENTER);
      if (lunaValid(c) && c < FRONT_GUARD_CM) { aborted = true; break; }
    }
    delay(10);
  }
  unsigned long elapsed = millis() - t0;
  stopDrive("");
  float y1 = getYaw();
  float d = y1 - y0;
  if (d >  180.0) d -= 360.0;
  if (d < -180.0) d += 360.0;
  outln(String("[pulse] done ") + (aborted ? "ABORTED" : "full") +
        " elapsed=" + String(elapsed) + "ms yaw1=" + String(y1, 1) +
        " d_yaw=" + String(d, 1));
  outln("        measure the straight-line move with a tape and log it against d_yaw");
  lastCmdMs = millis();
}

void arcPulse(int steerDir) {             // steerDir +1 = right, -1 = left
  steerTo(endpointFor(steerDir));
  delay(350);                             // let the servo reach the stop before rolling
  timedPulse(-1, ARC_PULSE_MS, steerDir > 0 ? "arc-rev-RIGHT" : "arc-rev-LEFT");
}

void setDuty(int d) {
  duty = constrain(d, 0, 255);
  if (driveDir != 0) setDrive(driveDir);       // apply live, keep direction
  outln("[duty] " + String(duty) + ((duty > 0 && duty < DUTY_MIN_USEFUL) ? "  (may stall)" : ""));
}

/* ============================ command dispatch ============================ */

void handle(char c) {
  lastCmdMs = millis();
  switch (c) {
    case 'w': setDrive(+1); outln("[cmd] forward");  break;
    case 's': setDrive(-1); outln("[cmd] reverse (UNGUARDED)"); break;
    case 'x': case ' ': stopDrive("manual"); break;

    case 'a': steerStep(-1); outln("[srv] " + String(servoAngle)); break;
    case 'd': steerStep(+1); outln("[srv] " + String(servoAngle)); break;
    case 'q': steerTo(endpointFor(-1)); outln("[srv] full LEFT "  + String(servoAngle)); break;
    case 'e': steerTo(endpointFor(+1)); outln("[srv] full RIGHT " + String(servoAngle)); break;
    case 'f': steerTo(SERVO_CENTER);    outln("[srv] centre "     + String(servoAngle)); break;

    case '[': setDuty(duty - DUTY_STEP); break;
    case ']': setDuty(duty + DUTY_STEP); break;
    case '1': setDuty(70);  break;
    case '2': setDuty(90);  break;
    case '3': setDuty(120); break;

    case 'z': yawRef = getYaw(); markN = 0;
              outln("[zero] yawRef=" + String(yawRef, 1)); break;
    case 'm': mark(); break;
    case 't': telOn = !telOn; outln(String("[tel] ") + (telOn ? "on" : "off")); break;
    case 'r': report(); break;
    case 'h': case '?': help(); break;

    case 'c': timedPulse(+1, PULSE_MS, "straight-fwd"); break;
    case 'v': timedPulse(-1, PULSE_MS, "straight-rev"); break;
    case 'o': arcPulse(-1); break;
    case 'p': arcPulse(+1); break;

    case 'g': frontGuard = !frontGuard;
              outln(String("[guard] ") + (frontGuard ? "on" : "OFF - forward is now unguarded")); break;
    case 'i': invertDrive = !invertDrive;
              if (driveDir != 0) setDrive(driveDir);
              outln("[invertDrive] " + String(invertDrive ? 1 : 0)); break;
    case 'k': invertSteering = !invertSteering;
              outln("[invertSteering] " + String(invertSteering ? 1 : 0) +
                    " - re-test q/e, then hardcode the value that matches reality"); break;
    default: break;                        // ignore noise, never act on it
  }
}

/* ============================ setup / loop ================================ */

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);          // TB6612 out of standby
  setMotorOutput(0);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP); // read-only here; resolves GPIO32 polarity

  Wire.begin();
  Wire.setClock(400000);

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  steerTo(SERVO_CENTER);

  selectMuxChannel(MUX_CH_BNO);
  // IMUPLUS: gyro+accel, no magnetometer - motor magnets cannot distort yaw
  if (!bno.begin(BnoCompat::imuplus())) {
    Serial.println("BNO055 MISSING on mux ch4 - yaw readings are dead, teleop still works");
  }
  delay(300);
  yawRef = getYaw();

  SerialBT.begin("TED_PARK_BENCH");
  outln("bench_teleop_park ready. BENCH TOOL - never flash this at the venue (rule 11.10).");
  help();
  report();
  lastCmdMs = millis();
}

void loop() {
  while (Serial.available())   handle((char)Serial.read());
  bool client = SerialBT.hasClient();
  while (client && SerialBT.available()) handle((char)SerialBT.read());

  // link loss = stop
  if (btWasClient && !client && driveDir != 0) stopDrive("bluetooth disconnected");
  btWasClient = client;

  // deadman
  if (driveDir != 0 && millis() - lastCmdMs > IDLE_STOP_MS) stopDrive("idle timeout");

  // forward guard — throttled to ~30 ms so the mux/Luna aren't hammered every loop
  static unsigned long lastGuardMs = 0;
  if (driveDir > 0 && frontGuard && millis() - lastGuardMs >= 30) {
    lastGuardMs = millis();
    int16_t c = getLunaDistance(MUX_CH_CENTER);
    if (lunaValid(c) && c < FRONT_GUARD_CM) {
      stopDrive(("front guard " + String(c) + "cm").c_str());
    }
  }

  if (telOn && millis() - lastTelMs >= TEL_PERIOD_MS) {
    lastTelMs = millis();
    outln(telLine("[tel]"));
  }
}
