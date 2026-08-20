/*
 * ROUND 2 (Obstacle Challenge) — built directly on the Round 1 sketch.
 * Base: round 1.ino @ main (heading-hold P-steer, corner = side opening,
 * TB6612 N20 module, GPIO32 start, 12-turn stop). All Round 1 logic kept;
 * new blocks are marked  // === R2 ===
 *
 * WHAT'S NEW
 *  1. Pi serial protocol (fresh — independent of any older Round 2 code):
 *     Pi sends newline-terminated lines over this same USB serial @115200:
 *       "R" or "RED"  = red block actionable
 *       "G" or "GREEN"= green block actionable
 *       "C" or "CLEAR"= nothing actionable
 *     Case-insensitive; any other line is ignored. Pi MUST re-send its current
 *     state at least every ~1 s (repeating is fine). No valid line for
 *     PI_TIMEOUT_MS -> treated as CLEAR (dead-man).
 *  2. Block on a STRAIGHT: light lane-change — the heading target is offset
 *     a few degrees toward the pass side (red->right, green->left), ramped
 *     slowly so the drift uses a lot of track; side lunas stop the drift
 *     before a wall.
 *  3. Block at a CORNER: wide vs narrow derived at runtime from
 *     colour x turn direction (pass side on the OUTSIDE of the turn = wide).
 *       wide   = keep straight WIDE_DELAY_MS past the opening, then turn 90
 *       narrow = turn 90 immediately + temporary extra cut into the corner
 *  4. Front-luna escape: front < ESC_TRIGGER_CM while a block is active ->
 *     stop, reverse straight, forward again steering toward the pass side.
 *  5. Parking-bay exit at start: rear touching the back magenta wall,
 *     bay = BAY_FACTOR (1.5) x BOT_LENGTH_CM, so free gap ahead =
 *     0.5 x bot length. Full-lock fwd/rev shuffle (parallel-park kinematics,
 *     IMU-gated) until the nose points EXIT_HEADING_DEG off the lane, drive
 *     out, then heading-hold straightens back onto the lane heading.
 *
 *  Finish behaviour is inherited from Round 1 unchanged (12 turns -> run-on
 *  -> brake). Parking / final run shape = phase 2, per Ethan.
 *
 * Wiring identical to Round 1:
 *   PCA9548A SDA->GPIO21 SCL->GPIO22 (0x70)   TF-Luna ch0/1/2   BNO055 ch4 (0x28)
 *   Servo->GPIO13, TB6612 AIN1=25 AIN2=26 PWMA=33 STBY=27, start button GPIO32
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

// ---------- Steering / turn tunables (Round 1 values) ----------
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

// === R2 === Pi link ---------------------------------------------------------
#define PI_TIMEOUT_MS     1500   // no R/G/C char for this long -> CLEAR (dead-man)

// === R2 === straight-line swerve --------------------------------------------
#define SWERVE_DEG        12.0f  // heading offset toward the pass side on a straight
#define SWERVE_RAMP_DEG   0.6f   // offset change per loop (~30 deg/s) = the "light" turn
#define WALL_MIN_CM       25     // side luna closer than this on the drift side -> stop drifting
#define STRAIGHT_ERR_DEG  20.0f  // only swerve when heading error < this (i.e. on a straight)
#define SWERVE_HOLD_MS    600    // keep the offset this long after the block leaves the
                                 // camera mid-pass — prevents cutting back into it

// === R2 === corner + block --------------------------------------------------
#define WIDE_DELAY_MS       450  // wide: keep straight this long past the opening, then turn
#define WIDE_ABORT_FRONT_CM 35   // during the wide delay: front closer than this -> turn NOW
#define NARROW_CUT_DEG      15.0f // narrow: temporary extra steer past the 90 target
#define NARROW_CUT_MS       900  // cut auto-expires after this (or once heading settles)

// === R2 === front-luna escape ----------------------------------------------
#define ESC_TRIGGER_CM    22     // front closer than this while a block is active -> escape
#define ESC_BRAKE_MS      150
#define ESC_REV_SPEED     650    // duty 0..1023, reverse leg
#define ESC_REV_MS        600    // reverse duration — BLIND (no rear sensor), keep short
#define ESC_FWD_MS        800    // forward leg with steering toward the pass side
#define ESC_TURN_DEG      30.0f  // heading offset used during the forward leg
#define ESC_COOLDOWN_MS   1200   // minimum gap between escapes
#define ESC_LIDAR_MUTE_MS 800    // no corner detection right after an escape

// === R2 === parking-bay exit (all variables — tune on the mat) --------------
#define BOT_LENGTH_CM     23.0f  // MEASURED 2026-08-20, bumper to bumper
#define BAY_FACTOR        1.5f   // bay length = BAY_FACTOR x bot length (rules)
#define BAY_GAP_CM        ((BAY_FACTOR - 1.0f) * BOT_LENGTH_CM)  // free space ahead ~= 11.5 cm
#define EXIT_DIR          +1     // +1 = track is to the RIGHT of the bot at start, -1 = LEFT.
                                 //     Set per round (drive direction) — needs a reflash.
#define EXIT_SPEED        520    // duty for the shuffle legs
#define EXIT_HEADING_DEG  45.0f  // rotate this far off the lane heading before driving out
// ⚠ TIME IS THE REAL GEOMETRY LIMIT IN THE BAY. With a 23 cm bot there are only
// ~11.5 cm ahead, and a full-lock 45° arc is ~35 cm of travel — so ONE forward leg
// can never reach the heading gate. Each leg must move ~8 cm and stop; the rotation
// accumulates over cycles. CALIBRATE: run the bot straight at EXIT_SPEED for 1.0 s,
// measure the distance, then EXIT_FWD_MS_MAX ~= 8 cm / (cm per second) * 1000.
#define EXIT_FWD_MS_MAX   400    // time cap per forward leg (~8 cm at an assumed ~20 cm/s)
#define EXIT_REV_MS       350    // per reverse leg — BLIND (no rear sensor), keep it < fwd
#define EXIT_FRONT_STOP_CM (BAY_GAP_CM * 0.55f)   // ~6.3 cm — last-resort front guard.
                                 // ⚠ MUST stay well BELOW BAY_GAP_CM: a value above the gap
                                 // (the old 15 vs 13/11.5) breaks the forward leg on its very
                                 // first check every cycle -> zero forward motion -> the bot
                                 // never leaves the bay. TF-Luna is unreliable under ~20 cm,
                                 // so this only ever fires opportunistically; time+heading rule.
#define EXIT_MAX_CYCLES   8      // fwd/rev shuffle cycles before giving up and driving out
#define EXIT_CLEAR_MS     900    // angled drive-out leg after the rotation
#define EXIT_LIDAR_MUTE_MS 1500  // bay walls must not register as corners while leaving

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDR, &Wire);
Servo servo;
float targetHeading = 0;   // the heading we steer to hold

// === R2 === shared state (promoted from loop() statics so exit/escape can set them)
unsigned long lidarResumeAt = 0;   // lunas paused until this millis()
bool leftWasOpen  = false;
bool rightWasOpen = false;

char blockCmd = 'C';               // last actionable Pi command: R / G / C
unsigned long lastPiMsgAt = 0;

float swerveOffset = 0;            // straight-line swerve (deg, + = steer right)
int   pendingTurnDir = 0;          // wide corner: 0 = none, else +1/-1 turn queued
unsigned long pendingTurnAt = 0;   // when the queued wide turn fires
int   narrowCutDir = 0;            // narrow corner: extra-cut direction
unsigned long narrowCutUntil = 0;  // extra cut active until this millis()
unsigned long escCooldownUntil = 0;
unsigned long lastBlockSeenAt  = 0;  // for the post-pass swerve hold

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

// === R2 === reject -1 and garbled reads in the NEW logic only.
// Round 1's corner compare on the raw value is left exactly as it was.
bool validDist(int d) { return d >= 2 && d <= 600; }

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
  servo.write((int)angle);
  return (int)angle;
}

// === R2 === Pi serial --------------------------------------------------------
// +1 = pass on the block's RIGHT (red), -1 = pass LEFT (green), 0 = none.
int passSideOf(char cmd) {
  if (cmd == 'R') return +1;
  if (cmd == 'G') return -1;
  return 0;
}

// Line-based on purpose: matching whole tokens means a stray word like
// "REVERSE" can never be misread as 'R' = red. Unknown lines are ignored.
void pollPi() {
  static char buf[12];
  static uint8_t n = 0;
  static bool discard = false;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (!discard && n > 0) {
        buf[n] = 0;
        char t = 0;
        if (!strcmp(buf, "R") || !strcmp(buf, "RED"))   t = 'R';
        if (!strcmp(buf, "G") || !strcmp(buf, "GREEN")) t = 'G';
        if (!strcmp(buf, "C") || !strcmp(buf, "CLEAR")) t = 'C';
        if (t) {
          if (t != blockCmd) Serial.printf("[pi] cmd=%c\n", t);
          blockCmd = t;
          lastPiMsgAt = millis();
        }
      }
      n = 0;
      discard = false;
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = (ch >= 'a' && ch <= 'z') ? ch - 32 : ch;   // uppercase as we go
    } else {
      discard = true;                              // overlong line -> drop whole line
    }
  }
  if (blockCmd != 'C' && millis() - lastPiMsgAt > PI_TIMEOUT_MS) {
    blockCmd = 'C';                                // dead-man: Pi went quiet
    Serial.println("[pi] timeout -> CLEAR");
  }
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

// ---------- Rule 9.11 wait-for-start (GPIO32 external button) ----------
#define START_BUTTON_PIN 32

void waitForStart() {
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  Serial.println("[start] waiting for start button (GPIO32 external)...");
  while (digitalRead(START_BUTTON_PIN) == HIGH) delay(10);   // wait for press
  delay(50);                                                 // debounce
  while (digitalRead(START_BUTTON_PIN) == LOW)  delay(10);   // wait for release
  Serial.println("[start] go - capturing heading reference");
}

// ============================================================================
// N20 drive motor via TB6612 (channel A) — unchanged from Round 1 except
// DRIVE_SPEED lowered for the obstacle round (R1 ran 1000).
// ============================================================================

#define MOTOR_AIN1         25
#define MOTOR_AIN2         26
#define MOTOR_PWMA         33
#define MOTOR_STBY         27      // set to -1 if STBY is hard-wired to 3V3
#define MOTOR_PWM_FREQ     20000
#define MOTOR_PWM_BITS     10      // duty range 0..1023
#define MOTOR_PWM_MAX      ((1 << MOTOR_PWM_BITS) - 1)
#define DRIVE_SPEED        700     // obstacle cruise duty — R1 used 1000; slower buys
                                   // the camera and the swerve geometry time. Tune.
#define MOTOR_INVERT       false
#define START_DELAY_MS     1500    // hands-off pause after the button
#define MAX_TURNS          12      // 3 laps x 4 corners
#define FINAL_RUN_MS       500     // run-on after the last turn (retune at DRIVE_SPEED 700)
#define TURN_SETTLED_DEG   15
#define SETTLE_TIMEOUT_MS  4000

ESP32PWM motorPwm;

void motorBrake() {
  digitalWrite(MOTOR_AIN1, HIGH);
  digitalWrite(MOTOR_AIN2, HIGH);
  motorPwm.write(0);
}

void motorCoast() {
  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, LOW);
  motorPwm.write(MOTOR_PWM_MAX);
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
    digitalWrite(MOTOR_STBY, HIGH);
  }
  motorPwm.attachPin(MOTOR_PWMA, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  motorBrake();
}

void trackTurnsAndStop() {
  static float prevTarget = NAN;
  static int turnCount = 0;
  static unsigned long finishAt = 0;
  static unsigned long lastTurnAt = 0;

  if (isnan(prevTarget)) prevTarget = targetHeading;

  if (targetHeading != prevTarget) {           // corner(s) taken
    turnCount += (int)(fabsf(targetHeading - prevTarget) / TURN_STEP_DEG + 0.5f);
    prevTarget = targetHeading;
    Serial.printf("[turn] %d/%d\n", turnCount, MAX_TURNS);
  }

  if (turnCount >= MAX_TURNS && finishAt == 0) {
    if (lastTurnAt == 0) lastTurnAt = millis();
    float err = fabsf(wrap180(readHeading() - targetHeading));
    if (err < TURN_SETTLED_DEG || millis() - lastTurnAt > SETTLE_TIMEOUT_MS) {
      finishAt = millis() + FINAL_RUN_MS;
      Serial.println("[turn] last turn settled - final run-on");
    }
  }
  if (finishAt != 0 && millis() >= finishAt) {
    motorBrake();
    Serial.println("Run complete - motor braked.");
    // TODO phase 2 (per Ethan): parking / final run shape goes here.
    while (1) delay(100);
  }
}

// === R2 === front-luna escape ------------------------------------------------
// Stop -> reverse straight (blind, short) -> forward steering toward the pass
// side of the block that triggered it -> resume cruise.
void doEscape() {
  int dir = passSideOf(blockCmd);              // guaranteed nonzero by the trigger gate
  Serial.printf("[esc] front blocked - stop/rev/turn dir=%+d\n", dir);

  motorBrake();
  delay(ESC_BRAKE_MS);

  servo.write(SERVO_CENTER);                   // reverse STRAIGHT — heading-hold is
  delay(120);                                  // geometry-inverted in reverse, skip it;
  setMotor(-ESC_REV_SPEED);                    // 120 ms lets the servo reach centre
  delay(ESC_REV_MS);
  motorBrake();
  delay(120);

  setMotor(DRIVE_SPEED);                       // forward, aimed toward the pass side
  unsigned long t0 = millis();
  while (millis() - t0 < ESC_FWD_MS) {
    float h = readHeading();
    steerToHeading(h, targetHeading + dir * ESC_TURN_DEG);
    delay(LOOP_DELAY_MS);
  }
                                               // offset released -> back to normal logic
  escCooldownUntil = millis() + ESC_COOLDOWN_MS;
  lidarResumeAt    = millis() + ESC_LIDAR_MUTE_MS;
  swerveOffset = 0;
  Serial.println("[esc] done - resuming");
}

// === R2 === parking-bay exit --------------------------------------------------
// Rear starts against the back magenta wall. Bay = BAY_FACTOR x bot length, so
// the free gap ahead is (BAY_FACTOR-1) x bot length (~0.5 L). Full-lock forward
// arcs toward the track alternated with opposite-lock reverse arcs keep the
// rotation going (parallel-park kinematics) until the nose points
// EXIT_HEADING_DEG off the lane; then drive out and hand back to heading-hold.
void bayExit() {
  const float ref = targetHeading;             // lane heading (captured at the button)
  Serial.printf("[exit] bot %.0f cm, free gap %.1f cm, front-stop %.1f cm, exit dir=%+d\n",
                (float)BOT_LENGTH_CM, (float)BAY_GAP_CM,
                (float)EXIT_FRONT_STOP_CM, (int)(EXIT_DIR));

  const int lockOut  = (EXIT_DIR > 0) ? SERVO_MAX_ANGLE : SERVO_MIN_ANGLE;
  const int lockBack = (EXIT_DIR > 0) ? SERVO_MIN_ANGLE : SERVO_MAX_ANGLE;

  for (int i = 0; i < EXIT_MAX_CYCLES; i++) {
    // forward arc toward the track
    servo.write(lockOut);
    delay(120);                                // let the servo reach lock first
    setMotor(EXIT_SPEED);
    unsigned long t0 = millis();
    while (millis() - t0 < EXIT_FWD_MS_MAX) {
      if (wrap180(readHeading() - ref) * (EXIT_DIR) >= EXIT_HEADING_DEG) break;
      int f = center();
      if (validDist(f) && f < EXIT_FRONT_STOP_CM) break;   // front bay wall
      delay(LOOP_DELAY_MS);
    }
    motorBrake();
    delay(80);
    if (wrap180(readHeading() - ref) * (EXIT_DIR) >= EXIT_HEADING_DEG) break;

    // reverse arc, opposite lock — rotation continues the same way.
    // BLIND: no rear sensor exists; EXIT_REV_MS must stay short.
    servo.write(lockBack);
    delay(120);
    setMotor(-EXIT_SPEED);
    t0 = millis();
    while (millis() - t0 < EXIT_REV_MS) {
      if (wrap180(readHeading() - ref) * (EXIT_DIR) >= EXIT_HEADING_DEG) break;
      delay(LOOP_DELAY_MS);
    }
    motorBrake();
    delay(80);
  }

  // angled drive-out: clear the bay walls at ref + EXIT_HEADING_DEG...
  setMotor(EXIT_SPEED);
  unsigned long t1 = millis();
  while (millis() - t1 < EXIT_CLEAR_MS) {
    float h = readHeading();
    steerToHeading(h, ref + EXIT_DIR * EXIT_HEADING_DEG);
    delay(LOOP_DELAY_MS);
  }

  // ...then hand back: heading-hold pulls the vehicle straight onto the lane.
  targetHeading = ref;
  lidarResumeAt = millis() + EXIT_LIDAR_MUTE_MS;   // bay walls are not corners
  while (Serial.available()) Serial.read();        // drop anything the Pi said so far
  blockCmd = 'C';
  Serial.println("[exit] out - resuming lane heading");
}

// ---------- Main ----------
void setup() {
  Serial.begin(115200);
  delay(300);
  initServo();
  delay(300);
  initIMU();
  initMotor();
  waitForStart();       // rule 9.11 no-touch start
  captureReference();   // heading reference captured AT the start press
  delay(START_DELAY_MS);
  bayExit();            // === R2 === leave the parking bay first
  Serial.println("\nReady: obstacle run - driving, swerving, turning.\n");
  setMotor(DRIVE_SPEED);
}

void loop() {
  pollPi();                                     // === R2 === drain Pi commands

  float h = readHeading();                      // always read heading, always steer

  bool detectMuted = (millis() < lidarResumeAt);   // gates corner DETECTION only —
  int l = left();                                  // the sensors themselves always run:
  int c = center();                                // the swerve wall-guard and the
  int r = right();                                 // escape need fresh reads every loop

  if (!detectMuted) {
    bool lOpen = (l > OPENING_CM);
    bool rOpen = (r > OPENING_CM);

    if (pendingTurnDir != 0) {
      // === R2 === a WIDE turn is queued: hold straight until the timer, or
      // bail into the turn early if the front wall arrives first.
      bool fireNow = (millis() >= pendingTurnAt);
      if (!fireNow && validDist(c) && c < WIDE_ABORT_FRONT_CM) {
        Serial.println("[corner] wide delay aborted by front wall - turning now");
        fireNow = true;
      }
      if (fireNow) {
        targetHeading += pendingTurnDir * TURN_STEP_DEG;
        lidarResumeAt = millis() + 1000;
        pendingTurnDir = 0;
        swerveOffset = 0;
      }
    } else {
      int dir = 0;
      if (lOpen && !leftWasOpen)  dir = -1;     // left opening -> left turn
      if (rOpen && !rightWasOpen) dir = +1;     // right opening -> right turn
      // (both first-edges in one loop: right wins — R1 cancelled to zero, a
      //  documented defect; here a turn must happen so one side is chosen)

      if (dir != 0) {
        int p = passSideOf(blockCmd);
        if (p == 0) {
          // no block: Round 1 behaviour, immediate 90
          targetHeading += dir * TURN_STEP_DEG;
          lidarResumeAt = millis() + 1000;
        } else if (p == dir) {
          // === R2 === NARROW: pass side is the inside of this turn.
          // Turn immediately + temporary extra cut into the corner.
          Serial.printf("[corner] narrow (%c, dir=%+d)\n", blockCmd, dir);
          targetHeading += dir * TURN_STEP_DEG;
          narrowCutDir   = dir;
          narrowCutUntil = millis() + NARROW_CUT_MS;
          lidarResumeAt  = millis() + 1000;
        } else {
          // === R2 === WIDE: pass side is the outside. Delay the 90 so the
          // arc swings out toward the outer wall before turning.
          Serial.printf("[corner] wide (%c, dir=%+d) - delaying turn %d ms\n",
                        blockCmd, dir, (int)WIDE_DELAY_MS);
          pendingTurnDir = dir;
          pendingTurnAt  = millis() + WIDE_DELAY_MS;
        }
        swerveOffset = 0;
      }
    }

    leftWasOpen  = lOpen;
    rightWasOpen = rOpen;
  }
  // detection muted: no corner edges fire — sensing, guards, steering stay live

  // === R2 === front-luna escape — ALWAYS armed (it has its own cooldown; a
  // block sitting right after a corner must trigger it even inside the mute)
  if (passSideOf(blockCmd) != 0 && validDist(c) && c < ESC_TRIGGER_CM &&
      millis() > escCooldownUntil && pendingTurnDir == 0) {
    doEscape();
    return;                                     // fresh loop after the maneuver
  }

  // === R2 === narrow-corner extra cut (expires by time or once settled)
  float cut = 0;
  if (millis() < narrowCutUntil) {
    if (fabsf(wrap180(h - targetHeading)) < TURN_SETTLED_DEG) narrowCutUntil = 0;
    else cut = narrowCutDir * NARROW_CUT_DEG;
  }

  // === R2 === straight-line swerve: light, ramped lane-change, wall-guarded
  int p = passSideOf(blockCmd);
  if (p != 0) lastBlockSeenAt = millis();
  float want = 0;
  if (p != 0 && pendingTurnDir == 0 &&
      fabsf(wrap180(h - targetHeading)) < STRAIGHT_ERR_DEG) {
    want = p * SWERVE_DEG;
  } else if (p == 0 && millis() - lastBlockSeenAt < SWERVE_HOLD_MS) {
    want = swerveOffset;      // block just left the camera mid-pass: hold the
  }                           // offset briefly so we don't cut back into it
  if (want > 0 && validDist(r) && r < WALL_MIN_CM) want = 0;   // wall on the right
  if (want < 0 && validDist(l) && l < WALL_MIN_CM) want = 0;   // wall on the left
  if (swerveOffset < want) {
    swerveOffset += SWERVE_RAMP_DEG; if (swerveOffset > want) swerveOffset = want;
  } else if (swerveOffset > want) {
    swerveOffset -= SWERVE_RAMP_DEG; if (swerveOffset < want) swerveOffset = want;
  }

  int servoAngle = steerToHeading(h, targetHeading + swerveOffset + cut);

  Serial.printf("L=%4d C=%4d R=%4d cm  H=%6.1f Tgt=%6.1f Sw=%+5.1f Servo=%3d cmd=%c %s\n",
                l, c, r, h, targetHeading, swerveOffset + cut, servoAngle, blockCmd,
                detectMuted ? "[detect muted]" : "");

  trackTurnsAndStop();
  delay(LOOP_DELAY_MS);
}
