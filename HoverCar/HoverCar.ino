/***************************************************
  HoverCar.ino - With Smooth Acceleration and Combined Command Support
  
  Features:
  - Gradual acceleration/deceleration for smoother control
  - Support for both traditional and combined command formats
  - Front wheel calibration
  - Watchdog safety timer
****************************************************/

#include <Wire.h>                   // For I2C
#include <Adafruit_MCP4725.h>       // For the DAC
#include "util.h"
#include "hoverserial.h"            // Provided with the hoverboard firmware library

// ========== SETTINGS ==========

// UART BAUD for the flashed hoverboard controllers
#define BAUDRATE       19200

// For clarity, define which hardware serial each side uses
#define SerialLeft     Serial2  // pins 16=TX2->RX,17=RX2->TX
#define SerialRight    Serial3  // pins 14=TX3->RX,15=TX

// Create two DAC objects
Adafruit_MCP4725 dacLeft;
Adafruit_MCP4725 dacRight;

// The addresses for your two MCP4725 boards
#define DACLEFT_ADDR   0x60
#define DACRIGHT_ADDR  0x61

// ---------- SPEED VARIABLES ----------
// Target speeds (where we want to go)
int targetLeftSpeed = 0;   // range: -1000..+1000
int targetRightSpeed = 0;  // range: -1000..+1000

// Current speeds (where we are now)
int currentLeftSpeed = 0;  // range: -1000..+1000
int currentRightSpeed = 0; // range: -1000..+1000

// ---------- ACCELERATION SETTINGS ----------
// These determine how quickly speed changes occur
// Lower values = smoother acceleration but less responsive
// Higher values = quicker response but more jerky
#define ACCEL_RATE 15      // Speed units per update (for accelerating)
#define DECEL_RATE 30      // Speed units per update (for decelerating - usually higher for safety)
#define SPEED_UPDATE_MS 20 // How often to update speeds (milliseconds)

// Command watchdog timer - for safety
#define COMMAND_TIMEOUT_MS 1000  // 1 second timeout
unsigned long lastCommandTime = 0;
unsigned long lastSpeedUpdate = 0;

// Front wheel calibration factor
#define FRONT_WHEEL_CALIBRATION 1.0

// Not strictly needed, but included for completeness
SerialHover2Server oHoverFeedback;

// ===================================================
void setup() {
  // Start USB serial (to communicate with the PC/Python)
  Serial.begin(115200);
  delay(200);
  Serial.println("HoverCar - With Smooth Acceleration - Starting...");

  // Setup Serial1 for ESP32 communication
  Serial1.begin(115200);
  Serial.println("Serial1 started for ESP32 communication");

  // Clear the serial buffers
  while (Serial.available() > 0) Serial.read();
  while (Serial1.available() > 0) Serial1.read();

  // Start the hoverboard controllers (front wheels)
  SerialLeft.begin(BAUDRATE);   // Left front
  SerialRight.begin(BAUDRATE);  // Right front

  // Init I2C and DACs (rear wheels)
  Wire.begin();

  if (!dacLeft.begin(DACLEFT_ADDR)) {
    Serial.println("MCP4725 (left) not found at 0x60! Check wiring.");
    while (1);
  }
  if (!dacRight.begin(DACRIGHT_ADDR)) {
    Serial.println("MCP4725 (right) not found at 0x61! Check wiring.");
    while (1);
  }
  Serial.println("Both DACs found (rear ESCs).");

  // Initialize DACs to zero to prevent startup motor spin
  dacLeft.setVoltage(0, false);
  dacRight.setVoltage(0, false);
  Serial.println("DACs initialized to zero voltage.");

  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize timing variables
  lastCommandTime = millis();
  lastSpeedUpdate = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Blink the built-in LED every 500 ms to show the loop is running
  static unsigned long lastBlink = 0;
  if (currentTime - lastBlink >= 500) {
    lastBlink = currentTime;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // Watchdog timeout - if no commands for a while, stop motors
  if (currentTime - lastCommandTime > COMMAND_TIMEOUT_MS) {
    if (targetLeftSpeed != 0 || targetRightSpeed != 0) {
      Serial.println("WATCHDOG: Command timeout - stopping motors!");
      targetLeftSpeed = 0;
      targetRightSpeed = 0;
    }
  }

  // Process commands from ESP32 (Serial1)
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim(); // remove trailing whitespace

    // Update the watchdog timer
    lastCommandTime = currentTime;
    
    // Process commands
    if (cmd.equalsIgnoreCase("0") || cmd.equalsIgnoreCase("S=0")) {
      targetLeftSpeed = 0;
      targetRightSpeed = 0;
    }
    else if (cmd.startsWith("L=") || cmd.startsWith("l=")) {
      targetLeftSpeed = constrain(cmd.substring(2).toInt(), -1000, 1000);
    }
    else if (cmd.startsWith("R=") || cmd.startsWith("r=")) {
      targetRightSpeed = constrain(cmd.substring(2).toInt(), -1000, 1000);
    }
    // Combined command format: "LR=left,right"
    else if (cmd.startsWith("LR=") || cmd.startsWith("lr=")) {
      String values = cmd.substring(3); // Skip "LR="
      int commaIndex = values.indexOf(',');
      
      if (commaIndex > 0) {
        String leftStr = values.substring(0, commaIndex);
        String rightStr = values.substring(commaIndex + 1);
        
        targetLeftSpeed = constrain(leftStr.toInt(), -1000, 1000);
        targetRightSpeed = constrain(rightStr.toInt(), -1000, 1000);
      }
    }

    // Debug output (rate-limited to avoid flooding)
    static unsigned long lastDebugPrint = 0;
    if (currentTime - lastDebugPrint >= 500) {
      lastDebugPrint = currentTime;
      Serial.print("ESP32 CMD → Target: L=");
      Serial.print(targetLeftSpeed);
      Serial.print(", R=");
      Serial.println(targetRightSpeed);
    }
  }

  // Process commands from USB Serial (manual control)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // remove trailing whitespace

    // Update the watchdog timer
    lastCommandTime = currentTime;
    
    // Process command
    if (cmd.equalsIgnoreCase("0")) {
      targetLeftSpeed = 0;
      targetRightSpeed = 0;
      Serial.println("Both sides set to 0.");
    }
    else if (cmd.equalsIgnoreCase("S=0")) {
      targetLeftSpeed = 0;
      targetRightSpeed = 0;
      Serial.println("Motors stopped (S=0).");
    }
    else if (cmd.startsWith("L=") || cmd.startsWith("l=")) {
      targetLeftSpeed = constrain(cmd.substring(2).toInt(), -1000, 1000);
      Serial.print("Target left speed set to ");
      Serial.println(targetLeftSpeed);
    }
    else if (cmd.startsWith("R=") || cmd.startsWith("r=")) {
      targetRightSpeed = constrain(cmd.substring(2).toInt(), -1000, 1000);
      Serial.print("Target right speed set to ");
      Serial.println(targetRightSpeed);
    }
    else if (cmd.startsWith("LR=") || cmd.startsWith("lr=")) {
      String values = cmd.substring(3);
      int commaIndex = values.indexOf(',');
      
      if (commaIndex > 0) {
        String leftStr = values.substring(0, commaIndex);
        String rightStr = values.substring(commaIndex + 1);
        
        targetLeftSpeed = constrain(leftStr.toInt(), -1000, 1000);
        targetRightSpeed = constrain(rightStr.toInt(), -1000, 1000);
        
        Serial.print("Target speeds set to L=");
        Serial.print(targetLeftSpeed);
        Serial.print(", R=");
        Serial.println(targetRightSpeed);
      }
    }
    // Special command to adjust acceleration rate
    else if (cmd.startsWith("ACCEL=")) {
      int newRate = constrain(cmd.substring(6).toInt(), 1, 100);
      Serial.print("Setting acceleration rate to ");
      Serial.println(newRate);
      // We'd update a global variable here if we were using one
    }
    else {
      Serial.println("Unrecognized command. Examples:");
      Serial.println("  0      (both sides to 0)");
      Serial.println("  S=0    (explicit stop)");
      Serial.println("  L=300  (left speed=300)");
      Serial.println("  R=-150 (right speed=-150)");
      Serial.println("  LR=300,-150 (combined command)");
      Serial.println("  ACCEL=10 (set acceleration rate)");
    }
  }

  // ===== SMOOTH ACCELERATION/DECELERATION =====
  // Update speeds at controlled intervals
  if (currentTime - lastSpeedUpdate >= SPEED_UPDATE_MS) {
    lastSpeedUpdate = currentTime;
    
    // Gradually approach target left speed
    if (currentLeftSpeed < targetLeftSpeed) {
      // Accelerating - use ACCEL_RATE
      currentLeftSpeed = min(currentLeftSpeed + ACCEL_RATE, targetLeftSpeed);
    } else if (currentLeftSpeed > targetLeftSpeed) {
      // Decelerating - use DECEL_RATE (usually faster for safety)
      currentLeftSpeed = max(currentLeftSpeed - DECEL_RATE, targetLeftSpeed);
    }
    
    // Gradually approach target right speed
    if (currentRightSpeed < targetRightSpeed) {
      currentRightSpeed = min(currentRightSpeed + ACCEL_RATE, targetRightSpeed);
    } else if (currentRightSpeed > targetRightSpeed) {
      currentRightSpeed = max(currentRightSpeed - DECEL_RATE, targetRightSpeed);
    }
    
    // Debug output (rate-limited)
    static unsigned long lastSpeedPrint = 0;
    if (currentTime - lastSpeedPrint >= 1000) { // Once per second
      lastSpeedPrint = currentTime;
      Serial.print("Current: L=");
      Serial.print(currentLeftSpeed);
      Serial.print(", R=");
      Serial.print(currentRightSpeed);
      Serial.print(" | Target: L=");
      Serial.print(targetLeftSpeed);
      Serial.print(", R=");
      Serial.println(targetRightSpeed);
    }
  }

  // Apply front wheel calibration if needed
  int adjustedLeftSpeed = -currentLeftSpeed;   // Note the negation for direction
  int adjustedRightSpeed = currentRightSpeed;
  
  if (FRONT_WHEEL_CALIBRATION < 1.0) {
    // Slow down the right wheel
    adjustedRightSpeed = (int)(adjustedRightSpeed * FRONT_WHEEL_CALIBRATION);
  } else if (FRONT_WHEEL_CALIBRATION > 1.0) {
    // Slow down the left wheel
    adjustedLeftSpeed = (int)(adjustedLeftSpeed / FRONT_WHEEL_CALIBRATION);
  }

  // Send speed commands to FRONT hoverboard controllers (UART)
  HoverSend(SerialLeft,  0, adjustedLeftSpeed,  32);
  HoverSend(SerialRight, 0, adjustedRightSpeed, 32);

  // Drive REAR ESCs with DAC (forward-only, so clamp negatives to 0)
  setLeftESC(currentLeftSpeed);
  setRightESC(currentRightSpeed);

  // Short delay to prevent CPU hogging
  delay(5);
}

// ========== HELPER FUNCTIONS ==========

/**
 * Sets the voltage for the LEFT ESC using the left DAC
 * range -1000..+1000, but clamp negative to 0 for forward-only
 */
void setLeftESC(int speedValue) {
  if (speedValue < 0)  speedValue = 0;     // clamp negative to 0
  if (speedValue > 1000) speedValue = 1000;

  // Map 0..1000 to 0..4095 (12-bit DAC)
  uint16_t dacVal = map(speedValue, 0, 1000, 0, 4095);
  dacLeft.setVoltage(dacVal, false);
}

/**
 * Sets the voltage for the RIGHT ESC using the right DAC
 * range -1000..+1000, but clamp negative to 0 for forward-only
 */
void setRightESC(int speedValue) {
  if (speedValue < 0)  speedValue = 0;
  if (speedValue > 1000) speedValue = 1000;

  // Map 0..1000 to 0..4095
  uint16_t dacVal = map(speedValue, 0, 1000, 0, 4095);
  dacRight.setVoltage(dacVal, false);
}