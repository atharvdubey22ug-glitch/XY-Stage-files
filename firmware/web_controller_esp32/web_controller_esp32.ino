/**
 * ============================================================
 *  FLEXURE STAGE CONTROLLER — ESP32 + 2x TMC2208 + 28BYJ-48
 *  v7 — non-blocking precision moves, move-complete acks,
 *       queued taps, hard halt
 * ============================================================
 *
 * QUICK-START: only touch the "TUNABLE PARAMETERS" section below.
 * Everything else is derived from those values automatically.
 *
 * SERIAL COMMANDS (115200 baud):
 *   R / L      — X axis right / left  (one tap = X_STEPS steps, queued)
 *   U / D      — Y axis up   / down
 *   + / -      — Z axis up   / down
 *   !          — HALT. Every axis stops dead, queues cleared.
 *   S:<val>    — Set Z speed / global default (e.g. S:80)
 *   J:<val>    — Set Z jog steps (e.g. J:1)
 *   C:<val>    — Set Z acceleration (e.g. C:500)
 *   M:<val>    — Set timeout ms (e.g. M:200)
 *   H:1 / H:0  — Hold coils on/off
 *   TX<n>      — Precision move X exactly n microsteps (e.g. TX1, TX-3)
 *   TY<n>      — Precision move Y exactly n microsteps
 *   TZ<n>      — Precision move Z exactly n microsteps
 *   TCX<n>     — Precision move X n microsteps WITH Y cross-axis compensation
 *   TCY<n>     — Precision move Y n microsteps WITH X cross-axis compensation
 *   XS:<val> / YS:<val>  — X / Y motor speed, steps/sec
 *   XA:<val> / YA:<val>  — X / Y motor acceleration, steps/s²
 *   XJ:<val> / YJ:<val>  — X / Y primary step size per tap
 *   XC:<ratio>           — X ratio fired alongside every Y tap (compensation, e.g. XC:0.12)
 *   YC:<ratio>           — Y ratio fired alongside every X tap (compensation, e.g. YC:-0.08)
 *   A:<val>    — Set flexure attenuation (e.g. A:0.82)
 *   MS:<val>   — Set microsteps display/calc value: 2, 4, 8 or 16 (pin-mode ceiling)
 *   P:<val>    — Set screw pitch, mm/rev (e.g. P:0.5)
 *   G:<val>    — Set gear ratio (e.g. G:64)
 *   N:<val>    — Set motor full steps/rev (e.g. N:32)
 *   B:<val>    — Set LED brightness, 0-255
 *   ?          — Print current settings
 *   #          — Print busy state of each axis
 *
 * ASYNCHRONOUS REPLIES:
 *   DONE:X / DONE:Y / DONE:Z are printed the moment an axis finishes
 *   everything queued for it. The host uses these to know exactly when
 *   the stage is at rest, instead of guessing from a timing model.
 *   HALT is printed in response to '!'.
 *
 * NOTHING BLOCKS.
 *   Serial is read character-by-character into a buffer, at most one
 *   command is handled per loop pass, and precision moves are executed
 *   by the same non-blocking run() calls as everything else. The old
 *   busy-wait inside the T handler made the board deaf for the whole
 *   duration of a move — a 400-step back-off at 50 steps/s locked it
 *   out for eight seconds, so '!' or a jog could not interrupt a run.
 *
 * INDEPENDENT ENABLE PINS:
 *   Each axis has its own EN pin. The motor for an axis only
 *   powers on when a command for that axis is received, and
 *   powers off after TIMEOUT_MS of inactivity on that axis.
 */

#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>

// AxisState must be defined here, right after the includes — Arduino's
// auto-generated function prototypes get inserted at this point in the
// file, so any struct used as a function parameter (enableAxis, jogAxis,
// etc.) must already exist by here or you'll get "was not declared in
// this scope" errors even though the struct is defined correctly below.
struct AxisState {
  AccelStepper* stepper;
  uint8_t        enPin;
  unsigned long  lastCmdTime;
  bool           enabled;
  bool           moving;      // a commanded move is in flight (drives DONE:)
  bool           continuous;  // timeout-mode free run, must be halted on timeout
  char           name;        // 'X' / 'Y' / 'Z', used in DONE: replies
  bool           report;      // announce MOVE:/DONE: for this move (host-driven moves only)
  unsigned long  moveId;      // token the host matches DONE: against
};

// ============================================================
//  PIN DEFINITIONS  (change if your wiring differs)
// ============================================================
#define X_STEP    26
#define X_DIR     27
#define Y_STEP    14
#define Y_DIR     12
#define Z_STEP    33
#define Z_DIR     32

// ── INDEPENDENT ENABLE PINS ──────────────────────────────────
// Wire each TMC2208's EN pin to its own GPIO.
// Both TMC2208 use ACTIVE LOW enable (LOW = on, HIGH = off).
// If sharing one driver for X+Z, tie EN_X and EN_Z to same pin.
#define EN_PIN_X  25    // X axis TMC2208 EN
#define EN_PIN_Y   4    // Y axis TMC2208 EN  ← change to your free GPIO
#define EN_PIN_Z  25    // Z axis — shares the X driver as wired today.
                        // NOTE: because this equals EN_PIN_X, powering Z
                        // down also cuts X. disableAxis() now refuses to
                        // drop a shared pin while any axis sharing it is
                        // still active — see the guard in disableAxis().

#define LED_PIN    5
#define NUM_LEDS   9

// ============================================================
//  TUNABLE PARAMETERS  — this is the section to play with
// ============================================================

// --- Motor speed (microsteps / second) ---
// Applies to Z, and is the default the X/Y values are seeded from.
// 28BYJ-48 reliably runs up to ~800-1000 at full step.
// At 8 microsteps (TMC2208 default) keep below 500.
// At 16 microsteps keep below 300.
float MOTOR_SPEED = 50.0;

// --- Timeout (milliseconds) ---
// How long after the last command on an axis before THAT axis powers off.
// Each axis has its own timeout timer — tapping X does not reset Y's timer.
// In timeout jog mode (JOG_STEPS = 0) this also halts the free run.
unsigned long TIMEOUT_MS = 350;

// --- Microstep setting on your TMC2208 (STANDALONE / pin mode) ---
// The TMC220x standalone table is NOT the A4988/DRV8825 ordering:
//   MS1=0, MS2=0 -> 1/8   (~25.0 nm/microstep at this stage geometry)
//   MS1=1, MS2=0 -> 1/2   (~100.1 nm)
//   MS1=0, MS2=1 -> 1/4   (~50.1 nm)
//   MS1=1, MS2=1 -> 1/16  (~12.5 nm)   <- CURRENT WIRING on the A4988-style
//                                         expansion board: both jumpers fitted
//                                         and reading high. Confirm with a
//                                         voltmeter or INDEX pulse count after
//                                         any reseat of the driver.
//
// 1/16 IS THE CEILING IN PIN MODE. There is no 32 or 256 setting here.
// microPlyer always interpolates whatever it is fed up to 256 internal
// microsteps, but that is smoothness, not resolution: it fills in the
// current waveform BETWEEN commanded steps and cannot be addressed. The
// smallest position you can command is still one step input. Real 1/256
// addressing needs the UART interface and the MRES register.
//
// DISPLAY/CALCULATION ONLY. This scales every nm figure the system reports,
// so if it disagrees with the actual pin wiring, every number in the data
// set is wrong by that factor. Verify the pins before a recording session.
int MICROSTEPS = 16;

// --- Screw pitch (mm per full output shaft revolution) ---
float SCREW_PITCH_MM = 0.5;

// --- Motor gear ratio (28BYJ-48 after bipolar conversion) ---
float GEAR_RATIO = 64.0;

// --- Motor full steps per revolution (after bipolar conversion) ---
float MOTOR_STEPS_PER_REV = 32.0;

// --- Flexure mechanical advantage ---
// Measured: ~0.82 near neutral, ~0.43 at higher deflection.
float FLEXURE_ATTENUATION = 0.82;

// --- Hold coils when idle? ---
// true  = motor stays powered after each tap (holds position, gets warm)
// false = motor powers off after TIMEOUT_MS (cool, may back-drive slightly)
// RECOMMENDATION: true eliminates startup jerk, and for repeatability runs
// it also removes an unmeasured settling transient between marks.
bool HOLD_WHEN_IDLE = true;

// --- Startup acceleration (steps/sec²) ---
float STARTUP_ACCEL = 200.0;

// --- Jog step size for Z (microsteps per tap) ---
// Set to 0 to fall back to timeout-based free-run mode.
int JOG_STEPS = 25;

// --- LED brightness (0–255) ---
uint8_t LED_BRIGHTNESS = 200;

// --- Direct per-axis diagonal-motion tuning ---
float X_SPEED  = 50.0;    // motor X steps/sec
float Y_SPEED  = 50.0;    // motor Y steps/sec
float X_ACCEL  = 200.0;   // motor X accel, steps/s²
float Y_ACCEL  = 200.0;   // motor Y accel, steps/s²
int   X_STEPS  = 25;      // primary step size per X tap (R/L)
int   Y_STEPS  = 25;      // primary step size per Y tap (U/D)
// Compensation as a RATIO of the primary move, not a fixed step count.
float Y_COMP_RATIO = 0.0;   // Y ratio fired alongside every X tap
float X_COMP_RATIO = 0.0;   // X ratio fired alongside every Y tap

// ============================================================
//  DERIVED VALUES  (auto-calculated, do not edit)
// ============================================================
float microsteps_per_output_rev() {
  return MOTOR_STEPS_PER_REV * GEAR_RATIO * MICROSTEPS;
}
float nm_per_microstep() {
  return (SCREW_PITCH_MM * 1000000.0) / microsteps_per_output_rev() * FLEXURE_ATTENUATION;
}
float nm_per_tap() {
  if (JOG_STEPS > 0) return nm_per_microstep() * JOG_STEPS;
  return nm_per_microstep() * (MOTOR_SPEED * TIMEOUT_MS / 1000.0);
}

// ============================================================
//  OBJECTS
// ============================================================
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP, Z_DIR);
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================
//  PER-AXIS STATE
// ============================================================
AxisState axisX = { &stepperX, EN_PIN_X, 0, false, false, false, 'X', false, 0 };
AxisState axisY = { &stepperY, EN_PIN_Y, 0, false, false, false, 'Y', false, 0 };
AxisState axisZ = { &stepperZ, EN_PIN_Z, 0, false, false, false, 'Z', false, 0 };

// Monotonic token for host-commanded moves. A bare "DONE:X" is ambiguous —
// an ack left over from an earlier jog can satisfy the wrong wait and let a
// measurement mark fire while the stage is still moving. The host matches
// MOVE:X:<id> to DONE:X:<id>, so that cannot happen.
unsigned long g_moveId = 0;

AxisState* allAxes[3] = { &axisX, &axisY, &axisZ };

// Non-blocking serial line assembly
String rxBuf = "";

// ============================================================
//  AXIS HELPERS
// ============================================================
void enableAxis(AxisState& ax) {
  if (!ax.enabled) {
    digitalWrite(ax.enPin, LOW);   // TMC2208 active LOW
    ax.enabled = true;
  }
  ax.lastCmdTime = millis();
}

// Powering down a shared EN pin would silently kill the other axis on it,
// leaving that axis flagged enabled while its driver is actually off — the
// next move would then be dropped with no error. Only drop the pin when
// every axis sharing it is idle.
bool sharedPinBusy(AxisState& ax) {
  for (auto other : allAxes) {
    if (other == &ax) continue;
    if (other->enPin == ax.enPin && other->enabled) return true;
  }
  return false;
}

void disableAxis(AxisState& ax) {
  if (sharedPinBusy(ax)) { ax.enabled = false; return; }  // logically off, pin stays live
  digitalWrite(ax.enPin, HIGH);
  ax.enabled = false;
}

// Compensation steps = ratio × signed primary steps, rounded to nearest
// microstep. Passing the SIGNED primary move means direction flips
// automatically — no separate negation needed at the call site.
long compSteps(float ratio, long primarySteps) {
  return lround(ratio * primarySteps);
}

// AccelStepper::move(n) is move-relative-to-WHERE-YOU-ARE-NOW, so calling it
// again mid-move throws away whatever was left of the previous move. Holding
// the D-pad sends a tap every 150 ms while one tap at 50 steps/s takes 500 ms,
// so two of every three taps were being silently discarded and the tap counter
// no longer matched the distance travelled. Queueing against the TARGET makes
// N taps always equal N × steps. Trade-off: the stage keeps running until the
// queue drains, so release the button slightly early — or use '!' to dump it.
void moveQueued(AxisState& ax, long steps, bool report) {
  if (steps == 0) return;
  ax.stepper->moveTo(ax.stepper->targetPosition() + steps);
  ax.moving = true;
  ax.continuous = false;
  if (report) {
    ax.report = true;
    ax.moveId = ++g_moveId;
    Serial.print("MOVE:"); Serial.print(ax.name);
    Serial.print(":");     Serial.println(ax.moveId);
  }
}

void jogAxis(AxisState& ax, int dir) {
  enableAxis(ax);
  ax.stepper->setMaxSpeed(MOTOR_SPEED);
  ax.stepper->setAcceleration(STARTUP_ACCEL);
  if (JOG_STEPS > 0) {
    moveQueued(ax, (long)dir * JOG_STEPS, false);
  } else {
    ax.stepper->move((long)dir * 99999);   // timeout-based free run
    ax.moving = true;
    ax.continuous = true;
  }
}

// ============================================================
//  SIMULTANEOUS DUAL-AXIS MOVE
//  Fires X and Y together, each with its own speed/accel/step
//  count. Both are non-blocking — the main loop's run() calls
//  advance them in lockstep, so this stays jog-responsive.
// ============================================================
void jogXY(long xSteps, float xSpeed, float xAccel,
           long ySteps, float ySpeed, float yAccel, bool report) {
  if (xSteps != 0) {
    enableAxis(axisX);
    stepperX.setMaxSpeed(xSpeed);
    stepperX.setAcceleration(xAccel);
    moveQueued(axisX, xSteps, report);
  }
  if (ySteps != 0) {
    enableAxis(axisY);
    stepperY.setMaxSpeed(ySpeed);
    stepperY.setAcceleration(yAccel);
    moveQueued(axisY, ySteps, report);
  }
}

// ============================================================
//  HARD HALT
//  Collapses every target onto the current position, so
//  distanceToGo() is zero on the very next run() call. No decel
//  ramp — at these speeds the geartrain will not lose a step,
//  and an emergency stop that ramps is not an emergency stop.
// ============================================================
void printPosition() {
  Serial.print("POS:X"); Serial.print(stepperX.currentPosition());
  Serial.print(",Y");    Serial.print(stepperY.currentPosition());
  Serial.print(",Z");    Serial.println(stepperZ.currentPosition());
}

void haltAll() {
  for (auto ax : allAxes) {
    ax->stepper->moveTo(ax->stepper->currentPosition());
    ax->stepper->setSpeed(0);
    ax->moving = false;
    ax->continuous = false;
  }
  Serial.println("HALT");
  printPosition();   // whoever halted us needs to know where we actually stopped
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Disable all axes at startup
  pinMode(EN_PIN_X, OUTPUT); digitalWrite(EN_PIN_X, HIGH);
  pinMode(EN_PIN_Y, OUTPUT); digitalWrite(EN_PIN_Y, HIGH);
  if (EN_PIN_Z != EN_PIN_X) {
    pinMode(EN_PIN_Z, OUTPUT); digitalWrite(EN_PIN_Z, HIGH);
  }

  for (auto ax : allAxes) {
    ax->stepper->setMaxSpeed(MOTOR_SPEED * 4.0);
    ax->stepper->setAcceleration(STARTUP_ACCEL);
  }

  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.clear();
  strip.show();

  printSettings();
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {

  // 1. Assemble incoming serial without blocking.
  //    readStringUntil('\n') stalls for the full Stream timeout (1 s by
  //    default) whenever a line arrives without its terminator — and every
  //    stepper freezes mid-move for that entire second. Read chars instead,
  //    and handle at most one command per pass so run() always gets called.
  bool handled = false;
  while (Serial.available() && !handled) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxBuf.length() > 0) {
        handleCommand(rxBuf);
        rxBuf = "";
        handled = true;
      }
    } else if (rxBuf.length() < 40) {
      rxBuf += c;
    }
    // over-length garbage is dropped silently rather than wrapping into
    // a half-command that would be misparsed as a move
  }

  unsigned long now = millis();

  // 2. Announce completions. The host waits on these instead of guessing
  //    from a speed/accel model, so marks can never land mid-move.
  for (auto ax : allAxes) {
    if (ax->moving && !ax->continuous && ax->stepper->distanceToGo() == 0) {
      ax->moving = false;
      if (ax->report) {
        ax->report = false;
        Serial.print("DONE:"); Serial.print(ax->name);
        Serial.print(":");     Serial.println(ax->moveId);
      }
    }
  }

  // 3. Per-axis timeout — each axis independently powers off
  for (auto ax : allAxes) {
    if (ax->enabled && (now - ax->lastCmdTime > TIMEOUT_MS)) {
      // A free-running timeout-mode jog never reaches distanceToGo()==0 on
      // its own (it was given 99999 steps), so it used to run forever and
      // the timeout never fired. Halt it here — this is what makes the
      // "safety timeout" in the web UI actually a safety timeout.
      if (ax->continuous) {
        ax->stepper->moveTo(ax->stepper->currentPosition());
        ax->continuous = false;
        ax->moving = false;
      }
      if (ax->stepper->distanceToGo() == 0 && !HOLD_WHEN_IDLE) {
        disableAxis(*ax);
      }
    }
  }

  // 4. Execute motion (non-blocking, uses acceleration ramp — no startup jerk)
  stepperX.run();
  stepperY.run();
  stepperZ.run();
}

// ============================================================
//  COMMAND HANDLER
// ============================================================
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // --- Emergency halt, checked first so it is never queued behind anything ---
  if (cmd == "!") { haltAll(); return; }

  // --- Axis movement ---
  if      (cmd == "R") { jogXY( X_STEPS, X_SPEED, X_ACCEL,  compSteps(Y_COMP_RATIO,  X_STEPS), Y_SPEED, Y_ACCEL, false); return; }
  else if (cmd == "L") { jogXY(-X_STEPS, X_SPEED, X_ACCEL,  compSteps(Y_COMP_RATIO, -X_STEPS), Y_SPEED, Y_ACCEL, false); return; }
  else if (cmd == "U") { jogXY( compSteps(X_COMP_RATIO,  Y_STEPS), X_SPEED, X_ACCEL,  Y_STEPS, Y_SPEED, Y_ACCEL, false); return; }
  else if (cmd == "D") { jogXY( compSteps(X_COMP_RATIO, -Y_STEPS), X_SPEED, X_ACCEL, -Y_STEPS, Y_SPEED, Y_ACCEL, false); return; }
  else if (cmd == "+") { jogAxis(axisZ,  1); return; }
  else if (cmd == "-") { jogAxis(axisZ, -1); return; }

  // --- On-the-fly speed: S:<val> ---
  else if (cmd.startsWith("S:")) {
    float v = cmd.substring(2).toFloat();
    if (v > 0 && v <= 1000) {
      MOTOR_SPEED = v;
      stepperZ.setMaxSpeed(MOTOR_SPEED);
      stepperZ.setAcceleration(STARTUP_ACCEL);
      Serial.print("Speed (Z / default): "); Serial.print(MOTOR_SPEED);
      Serial.print(" | nm/tap: "); Serial.println(nm_per_tap(), 1);
    } else Serial.println("Speed out of range (1-1000)");
  }

  // --- Jog steps: J:<val> ---
  else if (cmd.startsWith("J:")) {
    int v = cmd.substring(2).toInt();
    if (v >= 0 && v <= 10000) {
      JOG_STEPS = v;
      Serial.print("Jog steps (Z): "); Serial.print(JOG_STEPS);
      Serial.print(" | nm/tap: "); Serial.println(nm_per_tap(), 1);
    } else Serial.println("Jog steps out of range (0-10000)");
  }

  // --- Acceleration: C:<val> ---
  else if (cmd.startsWith("C:")) {
    float v = cmd.substring(2).toFloat();
    if (v >= 10 && v <= 5000) {
      STARTUP_ACCEL = v;
      stepperZ.setAcceleration(STARTUP_ACCEL);
      Serial.print("Accel (Z / default): "); Serial.println(STARTUP_ACCEL);
    } else Serial.println("Accel out of range (10-5000)");
  }

  // --- Timeout: M:<val> ---
  else if (cmd.startsWith("M:")) {
    unsigned long v = cmd.substring(2).toInt();
    if (v >= 50 && v <= 5000) {
      TIMEOUT_MS = v;
      Serial.print("Timeout: "); Serial.print(TIMEOUT_MS); Serial.println(" ms");
    } else Serial.println("Timeout out of range (50-5000)");
  }

  // --- Hold coils: H:1 or H:0 ---
  else if (cmd.startsWith("H:")) {
    HOLD_WHEN_IDLE = (cmd.charAt(2) == '1');
    Serial.print("Hold when idle: "); Serial.println(HOLD_WHEN_IDLE ? "YES" : "NO");
  }

  // --- Direct per-axis diagonal-motion tuning ---
  else if (cmd.startsWith("XS:")) {
    float v = cmd.substring(3).toFloat();
    if (v > 0 && v <= 1000) { X_SPEED = v; Serial.print("X speed: "); Serial.println(X_SPEED); }
    else Serial.println("X speed out of range (1-1000)");
  }
  else if (cmd.startsWith("YS:")) {
    float v = cmd.substring(3).toFloat();
    if (v > 0 && v <= 1000) { Y_SPEED = v; Serial.print("Y speed: "); Serial.println(Y_SPEED); }
    else Serial.println("Y speed out of range (1-1000)");
  }
  else if (cmd.startsWith("XA:")) {
    float v = cmd.substring(3).toFloat();
    if (v >= 10 && v <= 5000) { X_ACCEL = v; Serial.print("X accel: "); Serial.println(X_ACCEL); }
    else Serial.println("X accel out of range (10-5000)");
  }
  else if (cmd.startsWith("YA:")) {
    float v = cmd.substring(3).toFloat();
    if (v >= 10 && v <= 5000) { Y_ACCEL = v; Serial.print("Y accel: "); Serial.println(Y_ACCEL); }
    else Serial.println("Y accel out of range (10-5000)");
  }
  else if (cmd.startsWith("XJ:")) {
    int v = cmd.substring(3).toInt();
    if (v >= 0 && v <= 10000) { X_STEPS = v; Serial.print("X steps/tap: "); Serial.println(X_STEPS); }
    else Serial.println("X steps/tap out of range (0-10000)");
  }
  else if (cmd.startsWith("YJ:")) {
    int v = cmd.substring(3).toInt();
    if (v >= 0 && v <= 10000) { Y_STEPS = v; Serial.print("Y steps/tap: "); Serial.println(Y_STEPS); }
    else Serial.println("Y steps/tap out of range (0-10000)");
  }
  else if (cmd.startsWith("XC:")) {
    float v = cmd.substring(3).toFloat();
    if (v >= -3.0 && v <= 3.0) { X_COMP_RATIO = v; Serial.print("X comp ratio (fires with Y tap): "); Serial.println(X_COMP_RATIO, 4); }
    else Serial.println("X comp ratio out of range (-3.0-3.0)");
  }
  else if (cmd.startsWith("YC:")) {
    float v = cmd.substring(3).toFloat();
    if (v >= -3.0 && v <= 3.0) { Y_COMP_RATIO = v; Serial.print("Y comp ratio (fires with X tap): "); Serial.println(Y_COMP_RATIO, 4); }
    else Serial.println("Y comp ratio out of range (-3.0-3.0)");
  }

  // --- Flexure attenuation: A:<val> ---
  else if (cmd.startsWith("A:")) {
    float v = cmd.substring(2).toFloat();
    if (v > 0 && v <= 1.5) {
      FLEXURE_ATTENUATION = v;
      Serial.print("Flexure attenuation: "); Serial.println(FLEXURE_ATTENUATION, 3);
    } else Serial.println("Attenuation out of range (0-1.5)");
  }

  // --- Microsteps (display/calc only — physically set by MS1/MS2): MS:<val> ---
  else if (cmd.startsWith("MS:")) {
    int v = cmd.substring(3).toInt();
    if (v == 2 || v == 4 || v == 8 || v == 16) {
      MICROSTEPS = v;
      Serial.print("Microsteps (calc only, rewire MS1/MS2 to match): "); Serial.println(MICROSTEPS);
    } else Serial.println("Microsteps must be 2, 4, 8 or 16 in standalone pin mode");
  }

  // --- Screw pitch mm/rev: P:<val> ---
  else if (cmd.startsWith("P:")) {
    float v = cmd.substring(2).toFloat();
    if (v > 0 && v <= 5.0) {
      SCREW_PITCH_MM = v;
      Serial.print("Screw pitch: "); Serial.print(SCREW_PITCH_MM, 3); Serial.println(" mm/rev");
    } else Serial.println("Screw pitch out of range (0-5.0)");
  }

  // --- Gear ratio: G:<val> ---
  else if (cmd.startsWith("G:")) {
    float v = cmd.substring(2).toFloat();
    if (v > 0 && v <= 500) {
      GEAR_RATIO = v;
      Serial.print("Gear ratio: "); Serial.print(GEAR_RATIO, 2); Serial.println(":1");
    } else Serial.println("Gear ratio out of range (0-500)");
  }

  // --- Motor full steps/rev: N:<val> ---
  else if (cmd.startsWith("N:")) {
    float v = cmd.substring(2).toFloat();
    if (v > 0 && v <= 400) {
      MOTOR_STEPS_PER_REV = v;
      Serial.print("Motor steps/rev: "); Serial.println(MOTOR_STEPS_PER_REV, 1);
    } else Serial.println("Motor steps/rev out of range (0-400)");
  }

  // --- LED brightness: B:<val> ---
  else if (cmd.startsWith("B:")) {
    int v = cmd.substring(2).toInt();
    if (v >= 0 && v <= 255) {
      LED_BRIGHTNESS = (uint8_t)v;
      strip.setBrightness(LED_BRIGHTNESS);
      strip.show();
      Serial.print("LED brightness: "); Serial.println(LED_BRIGHTNESS);
    } else Serial.println("Brightness out of range (0-255)");
  }

  // --- Precision microstep move ---
  //   TX<n>  TY<n>  TZ<n>   raw single axis
  //   TCX<n> TCY<n>         same, with cross-axis compensation applied
  //
  // Raw moves deliberately skip compensation: a repeatability run wants to
  // see what the mechanism does on its own. Use the TC form when you want
  // to score whether a comp ratio actually cancels the parasitic diagonal —
  // run the same XY square both ways and compare closure error.
  //
  // These now use each axis's own configured speed and acceleration. The old
  // hard-coded setAcceleration(2000) meant the host could not predict how
  // long a move would take, and any settle time derived from the UI's
  // speed/accel sliders was wrong.
  else if (cmd.startsWith("T")) {
    bool comp   = (cmd.charAt(1) == 'C');
    char axis   = comp ? cmd.charAt(2) : cmd.charAt(1);
    long nsteps = cmd.substring(comp ? 3 : 2).toInt();

    if (nsteps == 0) { Serial.println("Precision move: zero steps ignored"); return; }

    if (comp && axis == 'X') {
      jogXY(nsteps, X_SPEED, X_ACCEL, compSteps(Y_COMP_RATIO, nsteps), Y_SPEED, Y_ACCEL, true);
    }
    else if (comp && axis == 'Y') {
      jogXY(compSteps(X_COMP_RATIO, nsteps), X_SPEED, X_ACCEL, nsteps, Y_SPEED, Y_ACCEL, true);
    }
    else if (axis == 'X') {
      enableAxis(axisX);
      stepperX.setMaxSpeed(X_SPEED); stepperX.setAcceleration(X_ACCEL);
      moveQueued(axisX, nsteps, true);
    }
    else if (axis == 'Y') {
      enableAxis(axisY);
      stepperY.setMaxSpeed(Y_SPEED); stepperY.setAcceleration(Y_ACCEL);
      moveQueued(axisY, nsteps, true);
    }
    else if (axis == 'Z') {
      enableAxis(axisZ);
      stepperZ.setMaxSpeed(MOTOR_SPEED); stepperZ.setAcceleration(STARTUP_ACCEL);
      moveQueued(axisZ, nsteps, true);
    }
    else { Serial.print("ERR:axis "); Serial.println(axis); return; }
    // No human-readable confirmation line here on purpose: MOVE:/DONE: is all
    // the host needs, and Serial.print blocks once the TX buffer fills, which
    // would stutter the step train during a run.
  }

  // --- Busy state ---
  else if (cmd == "#") {
    Serial.print("BUSY X:"); Serial.print(stepperX.distanceToGo());
    Serial.print(" Y:");     Serial.print(stepperY.distanceToGo());
    Serial.print(" Z:");     Serial.println(stepperZ.distanceToGo());
  }

  // --- Version handshake: the host refuses to record data from firmware
  //     older than this, because pre-v7 has no move acks. ---
  else if (cmd == "V") { Serial.println("VERSION:7"); }

  // --- Absolute step counters since boot ---
  // The host's idea of position is what it commanded. A halt mid-move makes
  // that a lie, so it queries this and corrects itself against the real
  // counters. Everything downstream of a resync is trustworthy again.
  else if (cmd == "@") { printPosition(); }

  // --- Print settings ---
  else if (cmd == "?") { printSettings(); }

  else { Serial.print("Unknown: "); Serial.println(cmd); }
}

// ============================================================
//  HELPERS
// ============================================================
void setLEDs(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++)
    strip.setPixelColor(i, strip.Color(r, g, b));
  strip.show();
}

void printSettings() {
  Serial.println("\n========================================");
  Serial.println("  FLEXURE STAGE CONTROLLER v7");
  Serial.println("  Non-blocking moves + DONE acks + halt");
  Serial.println("========================================");
  Serial.print("EN pins           : X="); Serial.print(EN_PIN_X);
  Serial.print("  Y="); Serial.print(EN_PIN_Y);
  Serial.print("  Z="); Serial.print(EN_PIN_Z);
  Serial.println(EN_PIN_Z == EN_PIN_X ? "  (Z shares X driver)" : "");
  Serial.print("Z speed / default : "); Serial.print(MOTOR_SPEED);    Serial.println(" steps/sec");
  Serial.print("Z jog steps       : "); Serial.println(JOG_STEPS);
  Serial.print("Z accel / default : "); Serial.print(STARTUP_ACCEL);  Serial.println(" steps/s2");
  Serial.print("Timeout           : "); Serial.print(TIMEOUT_MS);     Serial.println(" ms");
  Serial.print("Microsteps        : "); Serial.print(MICROSTEPS);     Serial.println("x");
  Serial.print("Screw pitch       : "); Serial.print(SCREW_PITCH_MM); Serial.println(" mm/rev");
  Serial.print("Flexure atten.    : "); Serial.println(FLEXURE_ATTENUATION, 2);
  Serial.print("X speed/accel/steps: "); Serial.print(X_SPEED); Serial.print(" / ");
  Serial.print(X_ACCEL); Serial.print(" / "); Serial.println(X_STEPS);
  Serial.print("Y speed/accel/steps: "); Serial.print(Y_SPEED); Serial.print(" / ");
  Serial.print(Y_ACCEL); Serial.print(" / "); Serial.println(Y_STEPS);
  Serial.print("Comp ratio (X on Y-tap) / (Y on X-tap): "); Serial.print(X_COMP_RATIO, 4);
  Serial.print(" / "); Serial.println(Y_COMP_RATIO, 4);
  Serial.println("--- Derived ---");
  Serial.print("nm per microstep  : "); Serial.print(nm_per_microstep(), 2); Serial.println(" nm");
  Serial.print("nm per tap (est.) : "); Serial.print(nm_per_tap(), 1);       Serial.println(" nm");
  Serial.print("Hold coils idle   : "); Serial.println(HOLD_WHEN_IDLE ? "YES (warm)" : "NO (cool)");
  Serial.println("========================================");
  Serial.println("R/L  X | U/D  Y | +/-  Z | !  HALT");
  Serial.println("S:<speed> | J:<steps> | C:<accel> | M:<ms> | H:1/0");
  Serial.println("XS:/YS: speed | XA:/YA: accel | XJ:/YJ: steps/tap");
  Serial.println("XC:<ratio> (X w/ Y-tap) | YC:<ratio> (Y w/ X-tap)");
  Serial.println("A:<atten> | MS:<2/4/8/16> | P:<pitch> | G:<ratio> | N:<steps> | B:<0-255>");
  Serial.println("TX<n> TY<n> TZ<n>   precision microstep, raw");
  Serial.println("TCX<n> TCY<n>       precision microstep, compensated");
  Serial.println("?  settings  |  #  busy state");
  Serial.println("Replies DONE:<axis> when an axis finishes, HALT on '!'");
  Serial.println("========================================\n");
}
