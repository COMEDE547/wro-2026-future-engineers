/*
 * motor_test.ino — standalone drive-motor bring-up sketch (ESP32)
 * ===============================================================
 * Verify wiring, direction and the speed cap over Serial BEFORE merging
 * motor control into the main driving firmware. Wheels off the ground.
 *
 * Build: this folder must contain motor_control.h (Arduino builds per-folder).
 * Set DRIVE_MODE + pins at the top of motor_control.h, upload, open Serial
 * Monitor @115200 (line ending: none / any).
 *
 * Keys:
 *   w / s   speed +0.10 / -0.10 (signed, so s past zero = reverse)
 *   1..9    set forward speed 10%..90% (clamped by DRIVE_MAX)
 *   r       reverse sign of current speed
 *   space/x coast stop        b   active brake
 *   i       print state
 *
 * If "forward" spins the wheels backward -> flip DRIVE_INVERT in
 * motor_control.h (do NOT rewire).
 */

#include <Arduino.h>
#include "motor_control.h"

static float cmd = 0.0f;
static unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nmotor_test: w/s +-0.1 | 1-9 speed | r reverse | space coast | b brake | i info");
  motorBegin();
  Serial.println("motor ready (wheels off the ground!)");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if      (c == 'w')              cmd += 0.10f;
    else if (c == 's')              cmd -= 0.10f;
    else if (c >= '1' && c <= '9')  cmd = 0.10f * (c - '0');
    else if (c == 'r')              cmd = -cmd;
    else if (c == ' ' || c == 'x') { cmd = 0.0f; motorStop(false); }
    else if (c == 'b')             { cmd = 0.0f; motorStop(true);  }
    else if (c == 'i')              lastPrint = 0;
    if (cmd >  1.0f) cmd =  1.0f;
    if (cmd < -1.0f) cmd = -1.0f;
  }

  if (cmd != 0.0f) motorDrive(cmd);   // keep feeding the watchdog while moving
  motorUpdate();

  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    Serial.printf("cmd=%+.2f  applied=%+.2f  (cap %.2f)\n",
                  cmd, motorCurrent(), DRIVE_MAX);
  }
  delay(20);
}
