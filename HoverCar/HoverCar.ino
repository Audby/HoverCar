/***************************************************
  HoverCar.ino - With Combined Command Support
  
  Updated to support both traditional L=/R= commands and
  the more efficient combined LR=L,R format
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

// Speed variables for left and right sides
int leftSpeed  = 0;  // range: -1000..+1000
int rightSpeed = 0;  // range: -1000..+1000

// Command watchdog timer - only used for safety
#define COMMAND_TIMEOUT_MS 1000  // 1 second timeout
unsigned long lastCommandTime = 0;

// Front wheel calibration factor
#define FRONT_WHEEL_CALIBRATION 1.0

// Not strictly needed, but included for completeness
SerialHover2Server oHoverFeedback;

// ===================================================
void setup() {
  // Start USB serial (to communicate with the PC/Python)
  Serial.begin(115200);
  delay(200);
  Serial.println("HoverCar - Combined Command Support - Starting...");

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

  // Fix for Issue 1: Explicitly initialize DACs to zero
  dacLeft.setVoltage(0, false);
  dacRight.setVoltage(0, false);
  Serial.println("DACs initialized to zero voltage.");

  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize watchdog timer
  lastCommandTime = millis();
}

void loop() {
  // Blink the built-in LED every 500 ms, just to show the loop is running
  static unsigned long lastBlink = 0;
  unsigned long now = millis();
  if (now - lastBlink >= 500) {
    lastBlink = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // Fix for Issue 2: Implement command timeout watchdog
  if (now - lastCommandTime > COMMAND_TIMEOUT_MS) {
    if (leftSpeed != 0 || rightSpeed != 0) {
      Serial.println("WATCHDOG: Command timeout - stopping motors!");
      leftSpeed = 0;
      rightSpeed = 0;
    }
  }

  // Check for commands from ESP32 (Serial1)
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim(); // remove trailing whitespace

    // Update the watchdog timer
    lastCommandTime = now;
    
    // Process command - support both formats
    if (cmd.equalsIgnoreCase("0") || cmd.equalsIgnoreCase("S=0")) {
      leftSpeed = 0;
      rightSpeed = 0;
    }
    else if (cmd.startsWith("L=") || cmd.startsWith("l=")) {
      leftSpeed = constrain(cmd.substring(2).toInt(), -1000, 1000);
    }
    else if (cmd.startsWith("R=") || cmd.startsWith("r=")) {
      rightSpeed = constrain(cmd.substring(2).toInt(), -1000, 1000);
    }
    // New combined command format: "LR=left,right"
    else if (cmd.startsWith("LR=") || cmd.startsWith("lr=")) {
      // Extract the combined values
      String values = cmd.substring(3); // Skip "LR="
      int commaIndex = values.indexOf(',');
      
      if (commaIndex > 0) {
        String leftStr = values.substring(0, commaIndex);
        String rightStr = values.substring(commaIndex + 1);
        
        // Parse the values
        leftSpeed = constrain(leftStr.toInt(), -1000, 1000);
        rightSpeed = constrain(rightStr.toInt(), -1000, 1000);
      }
    }

    // Print debug info but not too often to avoid serial flooding
    static unsigned long lastSerialPrint = 0;
    if (now - lastSerialPrint > 500) { // Print at most every 500ms
      lastSerialPrint = now;
      Serial.print("Command: ");
      Serial.print(cmd);
      Serial.print(" → L=");
      Serial.print(leftSpeed);
      Serial.print(", R=");
      Serial.println(rightSpeed);
    }
  }

  // Check for commands from USB Serial (for manual testing)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // remove trailing whitespace

    // Update the watchdog timer
    lastCommandTime = now;
    
    // Process command
    if (cmd.equalsIgnoreCase("0")) {
      leftSpeed = 0;
      rightSpeed = 0;
      Serial.println("Both sides set to 0.");
    }
    else if (cmd.equalsIgnoreCase("S=0")) {
      leftSpeed = 0;
      rightSpeed = 0;
      Serial.println("Motors stopped (S=0).");
    }
    else if (cmd.startsWith("L=") || cmd.startsWith("l=")) {
      leftSpeed = constrain(cmd.substring(2).toInt(), -1000, 1000);
      Serial.print("Left speed set to ");
      Serial.println(leftSpeed);
    }
    else if (cmd.startsWith("R=") || cmd.startsWith("r=")) {
      rightSpeed = constrain(cmd.substring(2).toInt(), -1000, 1000);
      Serial.print("Right speed set to ");
      Serial.println(rightSpeed);
    }
    else if (cmd.startsWith("LR=") || cmd.startsWith("lr=")) {
      // Process combined command
      String values = cmd.substring(3);
      int commaIndex = values.indexOf(',');
      
      if (commaIndex > 0) {
        String leftStr = values.substring(0, commaIndex);
        String rightStr = values.substring(commaIndex + 1);
        
        leftSpeed = constrain(leftStr.toInt(), -1000, 1000);
        rightSpeed = constrain(rightStr.toInt(), -1000, 1000);
        
        Serial.print("Speeds set to L=");
        Serial.print(leftSpeed);
        Serial.print(", R=");
        Serial.println(rightSpeed);
      }
    }
    else {
      Serial.println("Unrecognized command. Examples:");
      Serial.println("  0      (both sides to 0)");
      Serial.println("  S=0    (explicit stop)");
      Serial.println("  L=300  (left speed=300)");
      Serial.println("  R=-150 (right speed=-150)");
      Serial.println("  LR=300,-150 (combined command)");
    }
  }

  // Apply front wheel calibration if needed
  int adjustedLeftSpeed = -leftSpeed;   // Note the negation for direction
  int adjustedRightSpeed = rightSpeed;
  
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
  setLeftESC(leftSpeed);
  setRightESC(rightSpeed);

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