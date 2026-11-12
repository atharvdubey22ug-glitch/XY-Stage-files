// F²R XY Lab — ESP32 firmware (STEP/DIR, TMC2208 standalone)
//
// Current stage firmware (originally sketch_apr10b). Pin map matches
// README Step 3. The Nano version (firmware/xy_lab_nano) uses the same
// serial protocol.
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

// --- PIN DEFS ---
#define X_STEP 26
#define X_DIR  27
#define Y_STEP 14
#define Y_DIR  12
#define Z_STEP 33
#define Z_DIR  32
#define EN_PIN 25
#define LED_PIN 5
#define NUM_LEDS 9

// --- SETTINGS ---
// Adjust this speed value until it's smooth. 
// Higher = faster, but 28BYJ usually tops out around 800-1000.
float motorSpeed = 50; 
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 350; 

// --- OBJECTS ---
AccelStepper stepperX(1, X_STEP, X_DIR);
AccelStepper stepperY(1, Y_STEP, Y_DIR);
AccelStepper stepperZ(1, Z_STEP, Z_DIR);
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // Start DISABLED (Coils off/Cool)

  stepperX.setMaxSpeed(2000);
  stepperY.setMaxSpeed(2000);
  stepperZ.setMaxSpeed(2000);

  strip.begin();
  strip.clear();
  strip.show();
}

void loop() {
  // 1. Check for incoming Serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() > 0) {
      lastCommandTime = millis();
      digitalWrite(EN_PIN, LOW); // Power on the motors
      handleInput(cmd);
    }
  }

  // 2. SAFETY TIMEOUT (Instant Stop)
  // If no command received within TIMEOUT_MS, kill the speed immediately
  if (millis() - lastCommandTime > TIMEOUT_MS) {
    stepperX.setSpeed(0);
    stepperY.setSpeed(0);
    stepperZ.setSpeed(0);
    digitalWrite(EN_PIN, HIGH); // Power off coils to stop heat
  }

  // 3. EXECUTE MOTION (Non-blocking constant speed)
  stepperX.runSpeed();
  stepperY.runSpeed();
  stepperZ.runSpeed();
}

void handleInput(String cmd) {
  // X-Axis
  if (cmd == "R")      stepperX.setSpeed(motorSpeed);
  else if (cmd == "L") stepperX.setSpeed(-motorSpeed);
  
  // Y-Axis
  else if (cmd == "U") stepperY.setSpeed(motorSpeed);
  else if (cmd == "D") stepperY.setSpeed(-motorSpeed);
  
  // Z-Axis
  else if (cmd == "+") stepperZ.setSpeed(motorSpeed);
  else if (cmd == "-") stepperZ.setSpeed(-motorSpeed);

  // LED Controls
  else if (cmd == "1") setLEDs(255, 255, 255); 
  else if (cmd == "0") setLEDs(0, 0, 0);
}

void setLEDs(int r, int g, int b) {
  for(int i=0; i<NUM_LEDS; i++) strip.setPixelColor(i, strip.Color(r, g, b));
  strip.show();
}
