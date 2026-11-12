// F²R XY Lab — Arduino Nano firmware (STEP/DIR, TMC2208 standalone)
//
// Port of the ESP32 firmware (firmware/xy_lab_esp32) to the Nano pin map
// in README Step 3. Same serial protocol, so the same host software works
// with either controller.
//
// Serial, 115200 baud, one command per line:
//   R / L   X axis + / -
//   U / D   Y axis + / -
//   + / -   3rd (Z) axis + / -   (optional axis)
//   1 / 0   light ring on (white) / off
//
// Jog commands must be resent continuously. If no command arrives for
// TIMEOUT_MS, all axes stop and EN goes HIGH (drivers off, coils cool).
//
// Libraries: AccelStepper, Adafruit NeoPixel (both in Library Manager).

#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>

// --- PIN DEFS (Arduino Nano) ---
#define X_STEP  2
#define X_DIR   3
#define Y_STEP  4
#define Y_DIR   5
#define Z_STEP  6
#define Z_DIR   7
#define EN_PIN  8    // shared by all drivers, LOW = enabled
#define LED_PIN A0
#define NUM_LEDS 9

// --- SETTINGS ---
// Jog speed in steps/s. Higher = faster; the 28BYJ-48 tops out around 800-1000.
float motorSpeed = 50;
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 350;

// --- OBJECTS ---
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP, Z_DIR);
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void handleInput(const String &cmd);
void setLEDs(uint8_t r, uint8_t g, uint8_t b);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);       // don't stall stepping on a line without '\n'

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);  // start DISABLED (coils off)

  stepperX.setMaxSpeed(2000);
  stepperY.setMaxSpeed(2000);
  stepperZ.setMaxSpeed(2000);

  strip.begin();
  strip.clear();
  strip.show();
}

void loop() {
  // 1. Check for incoming serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() > 0) {
      lastCommandTime = millis();
      digitalWrite(EN_PIN, LOW);  // power on the motors
      handleInput(cmd);
    }
  }

  // 2. Safety timeout: no command within TIMEOUT_MS -> stop and disable
  if (millis() - lastCommandTime > TIMEOUT_MS) {
    stepperX.setSpeed(0);
    stepperY.setSpeed(0);
    stepperZ.setSpeed(0);
    digitalWrite(EN_PIN, HIGH);   // power off coils to stop heat
  }

  // 3. Execute motion (non-blocking constant speed)
  stepperX.runSpeed();
  stepperY.runSpeed();
  stepperZ.runSpeed();
}

void handleInput(const String &cmd) {
  // X axis
  if (cmd == "R")      stepperX.setSpeed(motorSpeed);
  else if (cmd == "L") stepperX.setSpeed(-motorSpeed);

  // Y axis
  else if (cmd == "U") stepperY.setSpeed(motorSpeed);
  else if (cmd == "D") stepperY.setSpeed(-motorSpeed);

  // Z / 3rd axis
  else if (cmd == "+") stepperZ.setSpeed(motorSpeed);
  else if (cmd == "-") stepperZ.setSpeed(-motorSpeed);

  // Light ring
  else if (cmd == "1") setLEDs(255, 255, 255);
  else if (cmd == "0") setLEDs(0, 0, 0);
}

void setLEDs(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, strip.Color(r, g, b));
  strip.show();
}
