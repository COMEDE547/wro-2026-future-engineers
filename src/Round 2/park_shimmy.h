#ifndef PARK_SHIMMY_H
#define PARK_SHIMMY_H
//
// =====================================================================
//  park_shimmy.h  --  magenta parallel-park controller (shimmy method)
// =====================================================================
//
//  STATUS: WRITTEN 2026-08-13. NEVER FLASHED. NEVER RUN ON A MAT.
//
//  This file lives on feat/round2-park ONLY. It must not be merged to
//  main until it has completed a park on the mat, because main's
//  docs/3_software.md and the engineering journal both state that no
//  state consumes MAG and that the park states are not in the committed
//  build. Merging untested states would make the repository contradict
//  itself on the surface a judge reads.
//
//  MANOEUVRE (as specified by the team, 2026-08-13):
//    1. Confirm the bay from the Pi's MAG stream.
//    2. Drive a short distance PAST the bay.
//    3. Reverse while steering toward the bay until the heading has
//       swung by one step.
//    4. Drive forward with counter-steer until the heading returns to
//       the reference (parallel to the wall).
//    5. Repeat 3-4 -- the "shimmy" -- walking the vehicle sideways
//       into the bay.
//    6. Stop when BOTH side rangefinders read short, i.e. the vehicle
//       is between the two limiters.
//
//  WHY HEADING AND NOT TIME:
//    Each leg terminates on a BNO055 heading delta, with a time cap only
//    as a backstop. Timing alone drifts with pack voltage; the IMU does
//    not. The time caps exist because of the constraint below.
//
//  *** THE BINDING CONSTRAINT: THERE IS NO REAR SENSOR. ***
//    All three TF-Lunas face left / centre / right. Every reverse leg is
//    therefore blind and dead-reckoned. This is why PARK_REV_MS is short,
//    why the angle step is small, and why the cycle count is capped.
//    Fitting a rear sensor would let the reverse legs close on distance
//    instead of time and is the single biggest improvement available to
//    this routine.
//
//  RULE 9.24.7: touching a lot limiter ENDS THE ROUND. Every constant
//    below is deliberately conservative. Rule 1.8.3 pays 7 points for a
//    crude or non-parallel park, so an aborted-but-stopped attempt still
//    scores; a contact does not. When in doubt this routine gives up and
//    stops rather than pressing for a tidier finish.
//
//  INTEGRATION (main.cpp):
//    #include "park_shimmy.h"                       // after the globals
//    ... in loop(), after the sensor block and before the normal
//        state dispatch:
//        if (parkActive()) { parkUpdate(); return; }
//    ... and where ROBOT_STOPPED is currently entered on turn 12:
//        parkArm();   // instead of stopping, hand over to the park
//
//  It calls only existing helpers: setMotorOutput(), steeringServo,
//  getCurrentHeading(), wrap360(), lunaValid(), the currentLeft/Center/
//  RightDist globals, the lastMag* globals and btPrint/btPrintln.
// =====================================================================

#define PARK_ENABLED 1     // 0 == compiled out entirely, legacy behaviour byte-identical

// ---------------------------------------------------------------- geometry
// Which side the bay is on. The lot sits against the OUTER wall of the
// start straight, so the bay is on the outside of the loop -- the
// opposite side to the turn direction locked at the first corner.
// true  = bay on the vehicle's RIGHT (counter-clockwise lap)
// BENCH-SET: confirm against the actual lap direction before running.
static bool  parkBayOnRight        = true;

// ---------------------------------------------------------------- tuning
// Every one of these is a starting value derived from geometry, not a
// mat-tuned number. They are all expected to move on the first session.
static const int   PARK_MAG_CONFIRM     = 4;      // MAG sightings before committing to a bay
static const unsigned long PARK_MAG_STALE_MS = 700;   // MAG older than this is not a sighting

static const int   PARK_SPEED_FWD       = 105;   // slow: precision beats speed here
static const int   PARK_SPEED_REV       = -105;

static const unsigned long PARK_PAST_MS = 850;   // how far past the bay to run before reversing
static const float PARK_ANGLE_STEP      = 24.0;  // deg of swing per reverse leg
static const float PARK_ANGLE_TOL       = 4.0;   // deg - "back to parallel" tolerance
static const unsigned long PARK_REV_MS  = 1300;  // HARD CAP per reverse leg (blind - no rear sensor)
static const unsigned long PARK_FWD_MS  = 1200;  // hard cap per forward leg
static const unsigned long PARK_SETTLE_MS = 250; // pause between legs so the IMU settles

static const int   PARK_IN_BAY_CM       = 30;    // both side Lunas below this => in the bay
static const int   PARK_FRONT_MIN_CM    = 14;    // centre Luna: never drive forward closer than this
static const int   PARK_MAX_CYCLES      = 6;     // give up after this many shimmies and stop where we are

// ---------------------------------------------------------------- state
enum ParkPhase {
  PARK_OFF = 0,   // not parking
  PARK_SEEK,      // armed, watching for a confirmed bay
  PARK_PAST,      // running past the bay
  PARK_REV,       // reverse leg, steering into the bay
  PARK_FWD,       // forward leg, counter-steer back to parallel
  PARK_CHECK,     // are both sides short yet?
  PARK_SETTLED,   // parked - terminal
  PARK_GIVEUP     // cycle cap hit - stopped, still scores under 1.8.3
};

static ParkPhase parkPhase      = PARK_OFF;
static int       parkMagHits    = 0;
static int       parkCycles     = 0;
static float     parkRefHeading = 0.0;
static unsigned long parkPhaseMs = 0;

static inline bool parkActive() {
#if PARK_ENABLED
  return parkPhase != PARK_OFF;
#else
  return false;
#endif
}

// Steer full-lock toward or away from the bay, respecting the asymmetric
// linkage window. Never writes outside SERVO_MAX_RIGHT..SERVO_MAX_LEFT.
static void parkSteer(bool towardBay) {
  bool right = parkBayOnRight ? towardBay : !towardBay;
  int  a     = right ? (INVERT_STEERING ? SERVO_MAX_LEFT : SERVO_MAX_RIGHT)
                     : (INVERT_STEERING ? SERVO_MAX_RIGHT : SERVO_MAX_LEFT);
  steeringServo.write(constrain(a, SERVO_MAX_RIGHT, SERVO_MAX_LEFT));
}

static void parkStraight() {
  steeringServo.write(SERVO_CENTER);
}

static void parkEnter(ParkPhase p) {
  parkPhase   = p;
  parkPhaseMs = millis();
}

// Arm the park routine. Called instead of entering ROBOT_STOPPED on the
// final lap when a park is wanted.
static void parkArm() {
#if PARK_ENABLED
  parkPhase   = PARK_SEEK;
  parkMagHits = 0;
  parkCycles  = 0;
  parkPhaseMs = millis();
  btPrintln("[park] armed - seeking bay");
#endif
}

// How far the heading has swung from the reference, signed-free.
static float parkHeadingDelta() {
  float d = wrap360(getCurrentHeading() - parkRefHeading);
  if (d > 180.0) d -= 360.0;
  return fabs(d);
}

static bool parkBothSidesShort() {
  return lunaValid(currentLeftDist)  && currentLeftDist  < PARK_IN_BAY_CM &&
         lunaValid(currentRightDist) && currentRightDist < PARK_IN_BAY_CM;
}

// Called every loop while parkActive(). Owns motor and servo entirely.
static void parkUpdate() {
#if PARK_ENABLED
  unsigned long now = millis();
  unsigned long inPhase = now - parkPhaseMs;

  switch (parkPhase) {

    // ---- watch for a confirmed bay -----------------------------------
    case PARK_SEEK: {
      parkStraight();
      setMotorOutput(PARK_SPEED_FWD);
      bool fresh = (lastMagX >= 0) && (now - lastMagMs < PARK_MAG_STALE_MS);
      if (fresh) {
        if (++parkMagHits >= PARK_MAG_CONFIRM) {
          parkRefHeading = getCurrentHeading();   // this heading IS "parallel to the wall"
          btPrintln("[park] bay confirmed - running past");
          parkEnter(PARK_PAST);
        }
      } else if (parkMagHits > 0 && (now - lastMagMs) > (PARK_MAG_STALE_MS * 2)) {
        parkMagHits = 0;                          // sighting went stale, start the count again
      }
      break;
    }

    // ---- drive a little past the bay ---------------------------------
    case PARK_PAST: {
      parkStraight();
      // never close on the wall ahead
      if (lunaValid(currentCenterDist) && currentCenterDist < PARK_FRONT_MIN_CM) {
        setMotorOutput(0);
        btPrintln("[park] wall ahead during run-past - starting shimmy early");
        parkEnter(PARK_REV);
        break;
      }
      setMotorOutput(PARK_SPEED_FWD);
      if (inPhase >= PARK_PAST_MS) {
        setMotorOutput(0);
        parkEnter(PARK_REV);
      }
      break;
    }

    // ---- reverse leg: swing the tail into the bay ---------------------
    // BLIND. No rear sensor. Terminates on heading swing, hard-capped on
    // time. Do not lengthen PARK_REV_MS without fitting a rear sensor.
    case PARK_REV: {
      parkSteer(true);                    // steer toward the bay
      setMotorOutput(PARK_SPEED_REV);
      if (parkHeadingDelta() >= PARK_ANGLE_STEP || inPhase >= PARK_REV_MS) {
        setMotorOutput(0);
        btPrint("[park] rev leg done, swing "); btPrintln(parkHeadingDelta());
        parkEnter(PARK_FWD);
      }
      break;
    }

    // ---- forward leg: counter-steer back to parallel ------------------
    case PARK_FWD: {
      if (inPhase < PARK_SETTLE_MS) { setMotorOutput(0); break; }
      parkSteer(false);                   // counter-steer
      if (lunaValid(currentCenterDist) && currentCenterDist < PARK_FRONT_MIN_CM) {
        setMotorOutput(0);
        btPrintln("[park] front guard - ending forward leg");
        parkEnter(PARK_CHECK);
        break;
      }
      setMotorOutput(PARK_SPEED_FWD);
      if (parkHeadingDelta() <= PARK_ANGLE_TOL || inPhase >= PARK_FWD_MS) {
        setMotorOutput(0);
        parkEnter(PARK_CHECK);
      }
      break;
    }

    // ---- are we in? ---------------------------------------------------
    case PARK_CHECK: {
      parkStraight();
      setMotorOutput(0);
      if (inPhase < PARK_SETTLE_MS) break;      // let the Lunas settle before judging

      if (parkBothSidesShort()) {
        btPrintln("[park] both sides short - parked");
        parkEnter(PARK_SETTLED);
        break;
      }
      if (++parkCycles >= PARK_MAX_CYCLES) {
        btPrintln("[park] cycle cap - stopping where we are (1.8.3 still pays)");
        parkEnter(PARK_GIVEUP);
        break;
      }
      btPrint("[park] cycle "); btPrint(parkCycles); btPrintln(" - shimmy again");
      parkEnter(PARK_REV);
      break;
    }

    // ---- terminal states ----------------------------------------------
    case PARK_SETTLED:
    case PARK_GIVEUP:
    default:
      parkStraight();
      setMotorOutput(0);
      break;
  }
#endif
}

#endif // PARK_SHIMMY_H
