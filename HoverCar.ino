/***************************************************
  TestAllWheels_DAC.ino - Two DAC version

  Hardware:
    - Arduino Mega
    - Two hoverboard controllers (front wheels), UART on Serial2 & Serial3
    - Two pot-based ESCs (rear wheels), driven by MCP4725 DAC modules at I2C addresses 0x60 and 0x61

  Usage:
    - Receives speed commands via USB Serial (Serial Monitor or from Python script).
    - Command format:
         L=NNN  -> set left side speed to NNN  (range -1000..+1000)
         R=NNN  -> set right side speed to NNN (range -1000..+1000)
         0      -> zero both sides
         S=0    -> explicit stop command
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

// Not strictly needed, but included for completeness
SerialHover2Server oHoverFeedback;

// ===================================================
void setup() {
  // Start USB serial (to communicate with the PC/Python)
  Serial.begin(115200);
  delay(200);
  Serial.println("TestAllWheels_DAC - Starting...");

  // Clear the serial buffer
  while (Serial.available() > 0) {
    Serial.read();
  }

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

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Blink the built-in LED every 500 ms, just to show the loop is running
  static unsigned long lastBlink = 0;
  unsigned long now = millis();
  if (now - lastBlink >= 500) {
    lastBlink = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // ========== 1) Check for commands from USB Serial ==========
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // remove trailing whitespace

    Serial.print("Received command: ");
    Serial.println(cmd);

    // "0" => zero both sides
    if (cmd.equalsIgnoreCase("0")) {
      leftSpeed  = 0;
      rightSpeed = 0;
      Serial.println("Both sides set to 0.");
    }
    // "S=0" => explicit stop
    else if (cmd.equalsIgnoreCase("S=0")) {
      leftSpeed  = 0;
      rightSpeed = 0;
      Serial.println("Motors stopped (S=0).");
    }
    // "L=NNN" => left side speed
    else if (cmd.startsWith("L=") || cmd.startsWith("l=")) {
      int val = cmd.substring(2).toInt();
      leftSpeed = val;
      Serial.print("Left speed set to ");
      Serial.println(leftSpeed);
    }
    // "R=NNN" => right side speed
    else if (cmd.startsWith("R=") || cmd.startsWith("r=")) {
      int val = cmd.substring(2).toInt();
      rightSpeed = val;
      Serial.print("Right speed set to ");
      Serial.println(rightSpeed);
    }
    else {
      Serial.println("Unrecognized command. Examples:");
      Serial.println("  0      (both sides to 0)");
      Serial.println("  S=0    (explicit stop)");
      Serial.println("  L=300  (left speed=300)");
      Serial.println("  R=-150 (right speed=-150)");
    }

    // clamp final speeds if we want to ensure ±1000 limit
    if (leftSpeed  > 1000)  leftSpeed  = 1000;
    if (leftSpeed  < -1000) leftSpeed  = -1000;
    if (rightSpeed > 1000)  rightSpeed = 1000;
    if (rightSpeed < -1000) rightSpeed = -1000;
  }

  // ========== 2) Send speed commands to FRONT hoverboard controllers (UART) ==========
  // Negative speeds can make them go in reverse if the flashed firmware supports it
  HoverSend(SerialLeft,  0, -leftSpeed,  32);
  HoverSend(SerialRight, 0,  rightSpeed, 32);

  // ========== 3) Drive REAR ESCs with DAC (forward-only, so clamp negatives to 0) ==========
  setLeftESC(leftSpeed);
  setRightESC(rightSpeed);

  // Print current speeds for debugging
  Serial.print("Current speeds: Left=");
  Serial.print(leftSpeed);
  Serial.print(", Right=");
  Serial.println(rightSpeed);

  // short delay to avoid flooding
  delay(20); // Reduced delay
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