#include <Wire.h>
#include <Adafruit_Sensor.h>
#define ENABLE_BLUETOOTH 0   // rule 11.10: radios must be OFF during rounds — leave 0 for competition builds
#if ENABLE_BLUETOOTH
#include "BluetoothSerial.h"
#endif
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>   // 2026-08-11 (research A5): task watchdog — see setup()

#define SERVO_PIN 13

#define MOTOR_IN1  25
#define MOTOR_IN2  26
#define MOTOR_PWM  33
#define MOTOR_STBY 27        // TB6612 STBY: driven HIGH; harmless if STBY is hardwired high on the breakout

#define START_BUTTON_PIN 32  // 2026-08-11: EXTERNAL start button on GPIO32 (per Ethan, supersedes onboard BOOT).
                             // Wiring assumption: button to GND, active LOW with internal pullup — matches the
                             // WAIT_FOR_START read below. BENCH-VERIFY: if the bot starts instantly or never,
                             // the button is wired to 3V3 instead — invert the digitalRead logic. (rules 9.10-9.11)

#define MUX_CH_LEFT   0
#define MUX_CH_CENTER 1  
#define MUX_CH_RIGHT  2
#define MUX_CH_BNO    4

#define TFLUNA_I2C_ADDR 0x10

float SLOW_STEER_THRESHOLD = 50.0; // Angle (in degrees) where the servo starts returning to center
int STRAIGHT_SPEED = 100;  // Cruise speed for driving straight (0-255)
int TURN_SPEED     = 100;  // Controlled speed for turning to prevent overshooting
int BACKWARD_SPEED = -100; // Speed for backing up (if needed)

// 2026-08-11 (research A4, MAT-TUNE): speed scheduling. Fast tier engages ONLY on a
// provably clear straight: heading settled, center Luna far, no turn cooldown, not
// avoiding. Conservative first-Nationals values; KILL SWITCH = set FAST_SPEED equal
// to STRAIGHT_SPEED and behavior is byte-identical to the single-speed build.
int   FAST_SPEED            = 130;   // clear-straight cruise (0-255)
const int   FAST_MIN_CENTER_CM  = 120;   // center Luna must see at least this much runway
const float FAST_MAX_HEADING_ERR = 4.0;  // deg — heading must be settled before speeding up


int SERVO_CENTER   = 106;   // Dead-center steering alignment — aligned 2026-08-10 to Round 1's mat-tuned value (58adb1c); same physical linkage, chassis final
int SERVO_MAX_LEFT  = 136;  // Physical linkage limit, higher-angle side (asymmetric about center: +30)
int SERVO_MAX_RIGHT = 64;   // Physical linkage limit, lower-angle side  (asymmetric about center: -42)
int DIFF = 42;              // Visual-swerve offset cap = the larger center-to-limit span; the per-side
                            // constrain(SERVO_MAX_RIGHT, SERVO_MAX_LEFT) below truncates the shorter (+30) side.
                            // 2026-08-10: was SERVO_CENTER±DIFF(35) symmetric — could not express the real
                            // 64/136 window (106+35=141 would command past the linkage). NOT yet mat-tested
                            // at these values: bench-verify steering direction (INVERT_STEERING) and center
                            // before flashing for a run.
                            // 2026-08-11: AUTHORITY ASYMMETRY (accepted + documented, no code change):
                            // center 106 sits off-middle of the 64..136 linkage window, so one avoidance
                            // side commands up to 42° of swerve while the other physically caps at 30° —
                            // those passes arc ~40% wider. No software fix exists (the linkage window is
                            // physical); if the mat shows clipping on the weak side, compensate by
                            // triggering that side's avoidance earlier on the Pi (safe-line/height
                            // thresholds), NOT by moving SERVO_CENTER — 106 is the mat-tuned
                            // straight-tracking value.

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

// ═══ PARALLEL PARKING (feat/round2-park — NOT for the 2026-08-12 scored push) ═══
// Rules 2026: the lot is ALWAYS in the starting section (Fig 8d), i.e. the section
// the vehicle re-enters when turn 12 completes. Lot = two magenta limiters
// 200x20x100 mm perpendicular to the OUTER wall, gap = 1.5 x vehicle length,
// depth 20 cm. 1.8.2 full parallel (projection inside + wheel-to-wall delta
// <= 2 cm) = 15 pts; 1.8.3 partial/not parallel = 7; touching a limiter ends the
// round with NO park points (9.24.7). Post-lap-3 the pillar side rules are OFF
// (App A sec.5) and signs in this section are moved toward the INNER wall (Fig 8e), so the
// outer-wall approach lane is pillar-free by construction.
// Every abort path degrades to a plain stop — points banked beat points risked.
// ⚠ ALL values below are MAT-UNVERIFIED until the Aug 22 bench.
// Rule cites verified verbatim against the official 2026 PDF (version Jan 15 2026):
// sec.8 + Fig 8d/8e, 9.24.4, 9.24.7, scoring table 1.8.1-1.8.3, 13.25-13.27 (limiter
// 200x20x100 magenta), App A sec.2 (30 s still), sec.5 (side rules off post-lap-3,
// moving still banned), sec.6 (parallel = wheel-to-wall delta <= 2 cm).
#define PARK_ENABLED 1              // 0 = byte-for-byte legacy stop behavior
const int           PARK_APPROACH_SPEED = 100;   // = STRAIGHT_SPEED; literal so tuning one can't silently move the other
const int           PARK_MAX_EXTRA_TURNS = 2;    // corners past 12 => we left the lot section: stop
const unsigned long PARK_MAG_WAIT_MS  = 6000;    // armed but no limiter ever seen -> legacy wall stop
const int           MAG_MIN_H         = 8;       // px; below this a MAG frame is noise, not a limiter
const int           MAG_NEAR_H        = 30;      // px; "close abeam" height — SET FROM [mag] LOGS ON THE MAT
const unsigned long MAG_FRESH_MS      = 450;     // > 2 periods of the Pi's 5 Hz MAG stream
const unsigned long MAG_LOST_MS       = 700;     // close-then-gone this long = passed abeam
const unsigned long MAG_SECOND_WAIT_MS = 2500;   // far limiter never re-acquired -> solo-passage mode
const float         MAG_HANDOFF_RATIO = 0.55;    // fresh h collapsing below ratio*max after close = near->far handoff (passage without dropout)
const unsigned long PARK_PASS_MS      = 900;     // nose-past-far-limiter run (~0.35 m at approach speed)
const unsigned long PARK_PASS_SOLO_MS = 1600;    // longer clearance when only one passage was seen
const float         PARK_CUT_IN_DEG   = 42.0;    // reverse-arc target yaw before counter-steer
const float         PARK_ALIGNED_DEG  = 8.0;     // |yaw error| considered parallel again
const unsigned long PARK_ARC_IN_MS    = 3000;    // hard caps: no arc runs forever
const unsigned long PARK_ARC_OUT_MS   = 3000;
const unsigned long PARK_ARC_MIN_MS   = 400;     // floor before ARC_OUT may declare PARKED (entry |dyaw| can already be < aligned tol)
const unsigned long PARK_TOTAL_MS     = 15000;   // arming -> forced stop (3-min round budget)
const int           PARK_WALL_NEAR_CM = 14;      // wall-side Luna floor during arc-in (diagonal read)
const int           PARK_WALL_STOP_CM = 5;       // wall-side Luna floor during arc-out (near-perpendicular)
const int SPIKE_PERSIST_LOOPS = 5;     // corner spike must hold this many consecutive loops (rejects pillar occlusion)

// gradient visual steering (tune on mat)
// 2026-08-11 (research A3, MAT-TUNE): per-color visual gains replace the single
// KV_VISUAL. Under the current config (INVERT_STEERING=true, center 106 in 64..136),
// RED avoidance steers toward the +30 side — 40% less physical authority than
// GREEN's -42 side — so red passes arced wider. RED's gain starts 1.2x as the mat-tune
// candidate; GREEN keeps the field-proven 0.28. RE-DERIVE which color is the weak side
// if INVERT_STEERING or SERVO_CENTER ever changes.
const float KV_VISUAL_RED   = 0.34f;   // deg of steer per px — weak (+30) side, boosted
const float KV_VISUAL_GREEN = 0.28f;   // deg of steer per px — strong (-42) side, unchanged
const int   MIN_ACTIVE_SWERVE = 8;     // deg floor while error > 0, guarantees progress
const int   SAFE_RED_X   = 90;         // block safe when cx <= this (matches Pi LEFT_SIDE_MAX)
const int   SAFE_GREEN_X = 150;        // block safe when cx >= this (matches Pi RIGHT_SIDE_MIN)

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo steeringServo;
#if ENABLE_BLUETOOTH
BluetoothSerial SerialBT;    // Added Bluetooth object
#endif

// PARK_* appended, never inserted: the serial protocol's state gates compare against
// named states, so RED/GREEN/CLEAR/REVERSE/POS are ignored in every PARK_* state with
// zero changes to the protocol block (post-lap-3 the side rules are off anyway, App A sec.5).
enum RobotState { WAIT_FOR_START, DRIVING_STRAIGHT, TURNING, ROBOT_STOPPED, OBSTACLE_AVOIDING, REVERSING, PARK_PASS, PARK_ARC_IN, PARK_ARC_OUT };
RobotState currentState = WAIT_FOR_START;

bool avoidDirectionRight = true;   // true = swerve right (for RED), false = swerve left (for GREEN)
unsigned long lastObstacleCmd = 0; // timeout safety
const unsigned long OBSTACLE_TIMEOUT_MS = 1500; // dead-man: auto-clear if no command for 1.5 s (Pi keeps state alive every 0.5 s)
unsigned long lastReverseCmd = 0;  // last time a REVERSE arrived
int lastPosX = -1;                 // latest block center-x from the Pi POS stream (0..239)
unsigned long lastPosMs = 0;       // when it arrived; 0 = none received this avoidance
unsigned long avoidStartMs = 0;    // 2026-08-11: when this avoidance began — bounds the blind full-lock fallback
int lastMagX = -1;                 // magenta limiter sighting from the Pi (MAG,cx,h[,w])
int lastMagH = -1;                 // consumed by the parking controller below when PARK_ENABLED;
int lastMagW = -1;                 // w = optional 3rd field (v3 pipeline parity), -1 when absent
unsigned long lastMagMs = 0;       // telemetry-only when PARK_ENABLED == 0

#if PARK_ENABLED
bool  parkArmed = false;           // set once when turn 12 completes
unsigned long parkArmedMs = 0;
int   parkTurnsAtArm = 0;
bool  parkLotOnRight = false;      // derived: CCW round (left turns) => outer wall on the right
float parkRefHeading = 0.0;        // wall-parallel cardinal, from the stepped-target chain (drift-free)
bool  magEstablished = false;      // limiter currently tracked
bool  magCloseSeen = false;        // tracked limiter got close (h >= MAG_NEAR_H)
int   magPassages = 0;             // limiters passed abeam (established-close-then-lost)
int   magMaxH = 0;                 // running max h since establish - handoff detection baseline
uint8_t magHandoffStreak = 0;      // consecutive fresh samples below the handoff ratio (2 = confirm)
unsigned long magPassageMs = 0;
bool  parkSoloPass = false;        // far limiter never re-acquired -> longer clearance run
unsigned long parkPhaseMs = 0;     // current PARK_* phase start
#endif

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


// 2026-08-11: TF-Luna emits 0 on signal loss and junk on bad reads — both previously
// passed the old (== -1) check. A Luna stuck at 0 for SPIKE_PERSIST_LOOPS loops faked a
// ±180 cm corner spike (phantom turn => the whole 90° target chain corrupts). Field
// diagonal is ~424 cm; anything outside 2..600 cm is not a real wall.
bool lunaValid(int16_t d) { return d >= 2 && d <= 600; }

// 2026-08-11 (research A6): median-of-3 per Luna channel — a single-frame 0/garbage
// (black wall, glancing angle, I2C hiccup) is outvoted by its two neighbours instead
// of reaching the spike logic; two-in-a-row still goes invalid and resets the streak
// via lunaValid. Cost: ~1 loop (~20-30 ms) of extra corner-detection lag on top of the
// 5-loop persistence — immaterial at these speeds.
int16_t lunaMedian3(uint8_t idx, int16_t v) {
  static int16_t h[3][3];
  static uint8_t n[3], p[3];
  h[idx][p[idx]] = v;
  p[idx] = (uint8_t)((p[idx] + 1) % 3);
  if (n[idx] < 3) { n[idx]++; return v; }
  int16_t a = h[idx][0], b = h[idx][1], c = h[idx][2];
  int16_t lo = min(a, min(b, c));
  int16_t hi = max(a, max(b, c));
  return (int16_t)((int32_t)a + b + c - lo - hi);
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
  // 2026-08-11: invalid now includes 0/garbage (lunaValid), and an invalid frame RESETS
  // the persistence streak — a phantom turn is catastrophic, a corner confirmed one
  // frame later is not (the spike persists the whole time the corner is actually there).
  if (!lunaValid(currentLeftDist) || !lunaValid(currentRightDist)) { spikeCount = 0; return; }

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
    // 2026-08-11: the blind full lock is now bounded. If POS never arrives within 600 ms
    // (tracker flicker), a committed max-deflection arc must not ride the 1.5 s dead-man
    // into a wall — relax toward center at ~2°/loop and let CLEAR/dead-man exit the state.
    if (millis() - avoidStartMs > 600) {
      if      (finalServoAngle > SERVO_CENTER) finalServoAngle = max(SERVO_CENTER, finalServoAngle - 2);
      else if (finalServoAngle < SERVO_CENTER) finalServoAngle = min(SERVO_CENTER, finalServoAngle + 2);
      steeringServo.write(finalServoAngle);
      return;
    }
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
    // 2026-08-11 (MAT-UNVERIFIED): was a hard hold of the last swerve angle until the
    // 1.5 s dead-man — up to ~1 s of blind committed arc after losing the block off-frame.
    // Now: 300–500 ms stale = hold unchanged (short flicker rides through exactly as
    // before); past 500 ms, relax toward center at ~2°/loop (~60°/s) so a genuinely lost
    // target straightens out instead of arcing into a wall.
    if (millis() - lastPosMs > 500) {
      if      (finalServoAngle > SERVO_CENTER) finalServoAngle = max(SERVO_CENTER, finalServoAngle - 2);
      else if (finalServoAngle < SERVO_CENTER) finalServoAngle = min(SERVO_CENTER, finalServoAngle + 2);
    }
    steeringServo.write(finalServoAngle);
    return;
  }

  int error;
  if (avoidDirectionRight) {
    error = lastPosX - SAFE_RED_X;      // RED: push the block left of the red safe line
  } else {
    error = SAFE_GREEN_X - lastPosX;    // GREEN: push the block right of the green safe line
  }
  float kv = avoidDirectionRight ? KV_VISUAL_RED : KV_VISUAL_GREEN;
  int offset = constrain((int)(kv * error), 0, DIFF);
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
  // 2026-08-11 (research A4): fast tier on provably clear straights. Uses the PREVIOUS
  // loop's headingError (one 20-30 ms loop of lag on the settled check — acceptable,
  // and it avoids reordering this function). Drops back to STRAIGHT_SPEED the moment
  // runway shortens, heading unsettles, or a turn cooldown is active; avoidance and
  // turning never see the fast tier (their handlers set their own speeds).
  int speed = STRAIGHT_SPEED;
  if (lunaValid(currentCenterDist) && currentCenterDist > FAST_MIN_CENTER_CM &&
      fabs(headingError) < FAST_MAX_HEADING_ERR && millis() > turnCooldownUntil) {
    speed = FAST_SPEED;
  }
#if PARK_ENABLED
  if (parkArmed) speed = PARK_APPROACH_SPEED;   // the lot approach never sees the fast tier
#endif
  setMotorOutput(speed);

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
  btPrint("[turn] "); btPrint(totalTurnsCount); btPrint("/"); btPrint(MAX_TURNS);
  btPrint(" target="); btPrintln(turnTargetHeading);
  steeringServo.write(SERVO_CENTER);
  finalServoAngle = SERVO_CENTER;
  straightTargetHeading = turnTargetHeading;
  currentState = DRIVING_STRAIGHT;
  turnCooldownUntil = millis() + TURN_COOLDOWN_MS;   // NEW
  }
}


#if PARK_ENABLED
// Same intent->endpoint mapping as avoidObstacle's field-proven full lock.
int fullLockToward(bool right) {
  if (right) return INVERT_STEERING ? SERVO_MAX_LEFT  : SERVO_MAX_RIGHT;
  else       return INVERT_STEERING ? SERVO_MAX_RIGHT : SERVO_MAX_LEFT;
}

// Signed yaw error vs the wall-parallel reference. RAW heading, matching the A11
// turn-completion convention: the EMA's ~200 ms lag would blow the arc exits.
float parkYawDelta() {
  float d = getCurrentHeading() - parkRefHeading;
  if (d > 180.0)  d -= 360.0;
  if (d < -180.0) d += 360.0;
  return d;
}

int16_t parkWallDist() { return parkLotOnRight ? currentRightDist : currentLeftDist; }

bool magFresh() {
  return lastMagMs != 0 && (millis() - lastMagMs) < MAG_FRESH_MS && lastMagH >= MAG_MIN_H;
}

void parkStop(const char* why) {
  btPrint("[park] STOP: "); btPrintln(why);
  setMotorOutput(0);
  steeringServo.write(SERVO_CENTER);
  finalServoAngle = SERVO_CENTER;
  currentState = ROBOT_STOPPED;
}
#endif

void printTelemetry(float currentHeading) {
  if (currentState == DRIVING_STRAIGHT) btPrint("MODE: STRAIGHT");
  else if (currentState == TURNING)     btPrint("MODE: TURNING ");
  else if (currentState == OBSTACLE_AVOIDING) btPrint("MODE: AVOIDING");
  else if (currentState == REVERSING)   btPrint("MODE: REVERSING");
#if PARK_ENABLED
  else if (currentState == PARK_PASS)    btPrint("MODE: PARK_PASS");
  else if (currentState == PARK_ARC_IN)  btPrint("MODE: PARK_ARC_IN");
  else if (currentState == PARK_ARC_OUT) btPrint("MODE: PARK_ARC_OUT");
#endif


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
  // 2026-08-11 (research A5): task watchdog, 3 s. A hung I2C/mux read or serial stall
  // now resets to WAIT_FOR_START (motors off) instead of freezing with the last motor
  // command still live — a reset ends the run; a freeze ends it into a wall.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdt_cfg = {};
  wdt_cfg.timeout_ms = 3000;
  wdt_cfg.idle_core_mask = 0;
  wdt_cfg.trigger_panic = true;
  esp_task_wdt_reconfigure(&wdt_cfg);   // core 3.x: framework already initialized the TWDT
#else
  esp_task_wdt_init(3, true);           // core 2.x signature
#endif
  esp_task_wdt_add(NULL);

  btPrintln("=== ROBOT READY - press START button ===");
  btPrintln("Commands: RED, GREEN, CLEAR, REVERSE, POS, MAG");
}


void loop() {

esp_task_wdt_reset();   // 2026-08-11: feed the watchdog first — covers every early-return branch

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
    //   MAG,cx,h  : any state (telemetry only — logged, never acted on)
    //   Everything is ignored during TURNING (never abort a corner), ROBOT_STOPPED
    //   (a post-finish detection must never restart the robot), and WAIT_FOR_START.
    if (serialBuffer == "RED") {
      if (currentState == DRIVING_STRAIGHT || currentState == OBSTACLE_AVOIDING) {
        if (currentState == DRIVING_STRAIGHT) { lastPosMs = 0; avoidStartMs = millis(); }  // fresh pillar: full lock until first POS
        avoidDirectionRight = true;
        currentState = OBSTACLE_AVOIDING;
        Serial.println("OBSTACLE: RED - steering right of the block");
        lastObstacleCmd = millis();
      }
    } 
    else if (serialBuffer == "GREEN") {
      if (currentState == DRIVING_STRAIGHT || currentState == OBSTACLE_AVOIDING) {
        if (currentState == DRIVING_STRAIGHT) { lastPosMs = 0; avoidStartMs = millis(); }  // fresh pillar: full lock until first POS
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
          // 2026-08-11: corrupted digits used to toInt() to 0 = a hard-left position
          // accepted as truth. Non-numeric tokens and cx outside the 0..239 frame are
          // line noise and the whole message is dropped (lastPosMs not refreshed).
          String tok = serialBuffer.substring(4, c1);
          bool numeric = tok.length() > 0;
          for (unsigned int i = 0; i < tok.length(); i++) { if (!isDigit(tok[i])) { numeric = false; break; } }
          int cx = numeric ? tok.toInt() : -1;
          if (numeric && cx <= 239) {
            lastPosX = cx;
            lastPosMs = millis();
            lastObstacleCmd = millis();   // the POS stream doubles as the keepalive
          }
        }
      }
    }
    else if (serialBuffer.startsWith("MAG,")) {
      // 2026-08-11 (research B1 foundation): magenta bay sighting from the Pi —
      // telemetry ONLY, accepted in every state precisely because it drives nothing.
      // Rate-limited [mag] lines land in the run log; Wednesday answers "is the bay
      // even reliably visible" before any parking controller is written.
      int c1 = serialBuffer.indexOf(',', 4);
      if (c1 > 4) {
        String tok = serialBuffer.substring(4, c1);
        bool numeric = tok.length() > 0;
        for (unsigned int i = 0; i < tok.length(); i++) { if (!isDigit(tok[i])) { numeric = false; break; } }
        if (numeric) {
          lastMagX = tok.toInt();
          int c2 = serialBuffer.indexOf(',', c1 + 1);   // v3: MAG,cx,h,w — w appended LAST,
          if (c2 > c1) {                                 // so this 2-field parse stays correct
            lastMagH = serialBuffer.substring(c1 + 1, c2).toInt();
            lastMagW = serialBuffer.substring(c2 + 1).toInt();
          } else {
            lastMagH = serialBuffer.substring(c1 + 1).toInt();
            lastMagW = -1;
          }
          lastMagMs = millis();
          static unsigned long lastMagPrint = 0;
          if (millis() - lastMagPrint >= 1000) {
            lastMagPrint = millis();
            btPrint("[mag] cx="); btPrint(lastMagX);
            btPrint(" h="); btPrint(lastMagH);
            btPrint(" w="); btPrintln(lastMagW);
          }
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
    // 2026-08-11 (research A7/B3): BNO055 IMUPLUS drift is fused-output drift — the
    // Nerdvana raw-rate bias subtraction does not transplant. Instrument first: while
    // the bot sits armed, log heading 1/s; Wednesday's logs characterize real drift
    // for free. Correction gets written ONLY if the data shows >2-3 deg over 3 min.
    static unsigned long lastDriftLog = 0;
    if (millis() - lastDriftLog >= 1000) {
      lastDriftLog = millis();
      btPrint("[drift] heading="); btPrintln(getCurrentHeading());
    }
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


  currentLeftDist   = lunaMedian3(0, getLunaDistance(MUX_CH_LEFT));
  currentCenterDist = lunaMedian3(1, getLunaDistance(MUX_CH_CENTER));
  currentRightDist  = lunaMedian3(2, getLunaDistance(MUX_CH_RIGHT));


  float currentHeading = getSmoothedHeading();


#if PARK_ENABLED
  // ── PARK ARMING. Turn 12 complete => the vehicle is re-entering the STARTING
  // section, which is where the lot always is (rule 8d). The stepped-target chain
  // makes straightTargetHeading the wall-parallel cardinal of this leg for free.
  if (!parkArmed && currentState == DRIVING_STRAIGHT && totalTurnsCount >= MAX_TURNS) {
    parkArmed      = true;
    parkArmedMs    = millis();
    parkTurnsAtArm = totalTurnsCount;
    parkLotOnRight = lockedDirectionLeft;   // left-turn (CCW) round => outer wall on the right
    parkRefHeading = straightTargetHeading;
    btPrint("[park] armed lotOnRight="); btPrint(parkLotOnRight ? 1 : 0);
    btPrint(" ref="); btPrintln(parkRefHeading);
  }

  // ── FALLBACKS. Every abort is a plain stop: rule 9.24.4 ends the round on a stop
  // after 3 laps (points banked), while touching a limiter scores zero (9.24.7).
  if (parkArmed && currentState != ROBOT_STOPPED) {
    if (millis() - parkArmedMs > PARK_TOTAL_MS)                       parkStop("total time cap");
    else if (totalTurnsCount > parkTurnsAtArm + PARK_MAX_EXTRA_TURNS) parkStop("left the lot section");
    else if (magPassages == 0 && !magEstablished &&
             millis() - parkArmedMs > PARK_MAG_WAIT_MS &&
             currentState == DRIVING_STRAIGHT &&
             lunaValid(currentCenterDist) && currentCenterDist < 165) parkStop("no limiter seen - legacy wall stop");
    if (currentState == ROBOT_STOPPED) return;
  }

  // ── LIMITER PASSAGES. The picker reports ONE magenta box, so the pair reads as two
  // sequential acquisitions: established -> close (h >= MAG_NEAR_H) -> lost = passed
  // abeam. Tracked only while DRIVING_STRAIGHT; a passage completing mid-avoidance is
  // seen late or missed, and the fallbacks price that in.
  if (parkArmed && currentState == DRIVING_STRAIGHT) {
    if (magFresh()) {
      if (!magEstablished) {
        magEstablished = true;
        magMaxH = lastMagH;
        magHandoffStreak = 0;
        btPrint("[park] limiter acquired h="); btPrintln(lastMagH);
      }
      if (lastMagH > magMaxH) magMaxH = lastMagH;
      if (lastMagH >= MAG_NEAR_H) magCloseSeen = true;
      // Near->far HANDOFF: the picker reports ONE box, so when the near limiter exits
      // frame while the far one is already confirmed, freshness never lapses and the
      // dropout path below cannot fire - the passage shows up as an h collapse instead.
      // Two consecutive sub-ratio samples (~400 ms at 5 Hz) reject single-frame noise.
      if (magCloseSeen && lastMagH < (int)(magMaxH * MAG_HANDOFF_RATIO)) {
        if (++magHandoffStreak >= 2) {
          magCloseSeen     = false;
          magMaxH          = lastMagH;
          magHandoffStreak = 0;
          magPassages++;
          magPassageMs = millis();
          btPrint("[park] passage "); btPrint(magPassages); btPrintln(" (handoff)");
          if (magPassages >= 2) {
            parkSoloPass = false;
            parkPhaseMs  = millis();
            currentState = PARK_PASS;
            btPrintln("[park] PASS (both limiters)");
          }
        }
      } else {
        magHandoffStreak = 0;
      }
    } else if (magEstablished && millis() - lastMagMs > MAG_LOST_MS) {
      magEstablished = false;
      magMaxH = 0;
      magHandoffStreak = 0;
      if (magCloseSeen) {
        magCloseSeen = false;
        magPassages++;
        magPassageMs = millis();
        btPrint("[park] passage "); btPrintln(magPassages);
        if (magPassages >= 2) {
          parkSoloPass = false;
          parkPhaseMs  = millis();
          currentState = PARK_PASS;
          btPrintln("[park] PASS (both limiters)");
        }
      }
    }
    // Far limiter never re-acquired (merged blob at distance, or missed): commit on the
    // near passage alone with the longer clearance run.
    if (currentState == DRIVING_STRAIGHT &&
        magPassages == 1 && !magEstablished &&
        millis() - magPassageMs > MAG_SECOND_WAIT_MS) {
      parkSoloPass = true;
      parkPhaseMs  = millis();
      currentState = PARK_PASS;
      btPrintln("[park] PASS (solo passage)");
    }
  }
#else
  // 2026-08-11: stop check now gated on DRIVING_STRAIGHT — it previously ran in every
  // state, so after lap 3 a pillar at <165 cm (or a mid-swerve diagonal wall sighting)
  // stopped the bot yawed, mid-avoidance, instead of settled on the final straight. A
  // final-straight pillar is now avoided first; the wall stop fires after CLEAR.
  if (currentState == DRIVING_STRAIGHT &&
      totalTurnsCount >= MAX_TURNS && lunaValid(currentCenterDist) && currentCenterDist < 165) {
    btPrint("[stop] turns="); btPrint(totalTurnsCount);
    btPrint(" center_cm="); btPrintln(currentCenterDist);
    currentState = ROBOT_STOPPED;
    return;
  }
#endif


  if (currentState == DRIVING_STRAIGHT) {
    checkObstacles();
    driveStraightMode(currentHeading);
  }

  else if (currentState == OBSTACLE_AVOIDING) {
    // 2026-08-11 (MAT-UNVERIFIED): corner detection now stays armed during avoidance.
    // Before this, a spike arriving mid-avoid was invisible — the bot kept visual-steering
    // toward the corner wall until CLEAR (~0.5 s Pi debounce) plus a 5-loop re-arm. A
    // corner turn taken beside a pillar risks brushing it (penalty); driving the corner
    // wall ends the run — the turn is strictly the cheaper failure. Stepped targets keep
    // the turn geometry correct from any swerved pose (the target derives from the leg
    // reference heading, not the current heading). Mat test before trusting: pillar
    // placed at a corner entry, both colors, both directions.
    checkObstacles();
    if (currentState == TURNING) {
      lastPosMs = 0;   // corner cancels the avoidance handshake; Pi's CLEAR (ignored
                       // during TURNING) resets its side while we execute the turn
    } else {
      avoidObstacle();
    }
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
#if PARK_ENABLED
  else if (currentState == PARK_PASS) {
    // Heading-held straight run so the REAR AXLE clears the far limiter before the arc.
    // The outer-wall lane is pillar-free here (Fig 8e moves this section's signs to the
    // inner wall) and side rules are off post-lap-3 anyway (App A sec.5). parkArmed pins the speed.
    driveStraightMode(currentHeading);
    if (millis() - parkPhaseMs > (parkSoloPass ? PARK_PASS_SOLO_MS : PARK_PASS_MS)) {
      parkPhaseMs  = millis();
      currentState = PARK_ARC_IN;
      btPrint("[park] ARC_IN dyaw="); btPrintln(parkYawDelta());
    }
  }
  else if (currentState == PARK_ARC_IN) {
    // Reverse, full lock TOWARD the wall: the tail swings into the lot. Exits on the
    // cut-in angle; the wall-side Luna (diagonal, best-effort on black) and the time
    // cap both cut the arc short — proceeding to ARC_OUT is always the safe branch.
    finalServoAngle = fullLockToward(parkLotOnRight);
    steeringServo.write(finalServoAngle);
    setMotorOutput(BACKWARD_SPEED);
    float d = parkYawDelta();
    int16_t w = parkWallDist();
    static unsigned long lastArcLog = 0;
    if (millis() - lastArcLog >= 200) { lastArcLog = millis();
      btPrint("[park] in dyaw="); btPrint(d); btPrint(" wall="); btPrintln(w); }
    if (fabs(d) >= PARK_CUT_IN_DEG ||
        (lunaValid(w) && w < PARK_WALL_NEAR_CM) ||
        millis() - parkPhaseMs > PARK_ARC_IN_MS) {
      parkPhaseMs  = millis();
      currentState = PARK_ARC_OUT;
      btPrint("[park] ARC_OUT dyaw="); btPrintln(d);
    }
  }
  else if (currentState == PARK_ARC_OUT) {
    // Still reversing, counter lock: tail settles along the wall, nose swings in.
    // Aligned-within-tolerance is the parked exit; wall floor and time cap stop short
    // (a crooked stop in the lot is still 7 points, rule 1.8.3).
    finalServoAngle = fullLockToward(!parkLotOnRight);
    steeringServo.write(finalServoAngle);
    setMotorOutput(BACKWARD_SPEED);
    float d = parkYawDelta();
    int16_t w = parkWallDist();
    static unsigned long lastArcLog2 = 0;
    if (millis() - lastArcLog2 >= 200) { lastArcLog2 = millis();
      btPrint("[park] out dyaw="); btPrint(d); btPrint(" wall="); btPrintln(w); }
    if (millis() - parkPhaseMs > PARK_ARC_MIN_MS &&
        fabs(d) <= PARK_ALIGNED_DEG)                    parkStop("PARKED - attempt complete");
    else if (lunaValid(w) && w < PARK_WALL_STOP_CM)     parkStop("wall floor");
    else if (millis() - parkPhaseMs > PARK_ARC_OUT_MS)  parkStop("arc-out cap");
  }
#endif


  static uint8_t telemetryDiv = 0;   // A12: telemetry every 5th loop — a full line costs ~10 ms of serial time
  if (++telemetryDiv >= 5) {
    telemetryDiv = 0;
    printTelemetry(currentHeading);
  }
  delay(20);
}


