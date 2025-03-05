/*********************************************************************************
  ESP32 Sketch using Bluepad32 to read PS5 controller, then send commands
  to Arduino Mega for your hoverboard car.

  Optimized with:
  - Combined commands to reduce serial overhead
  - Improved timing and reduced unnecessary delays 
  - Only sends commands when values change or after time interval
  - Enhanced deadzones to prevent drift
**********************************************************************************/

#include <Arduino.h>
#include <Bluepad32.h>
#include <HardwareSerial.h>

// CREATE A HARDWARE SERIAL INTERFACE FOR MEGA
HardwareSerial SerialMega(2); // "2" = UART2 instance

// Bluepad32 can connect up to 4 controllers.
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Track the controller connection
bool wasConnected = false;

// Current and previous values for change detection
int prevLeftVal = 0;
int prevRightVal = 0;
unsigned long lastCommandTime = 0;

// Deadzone settings
#define THROTTLE_DEADZONE 70    // Deadzone for throttle/brake (0-1023)
#define STEERING_DEADZONE 70    // Deadzone for steering axis (-512 to 511)

// Timing settings
#define MIN_COMMAND_INTERVAL_MS 40  // Only send commands at most every X ms
#define FORCE_COMMAND_INTERVAL_MS 200  // Force sending commands every X ms even if values haven't changed

// Called when a new controller connects
void onConnectedController(ControllerPtr ctl) {
  Serial.println("Controller connected!");
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      Serial.printf("Assigned to slot %d\n", i);
      break;
    }
  }
}

// Called when a controller disconnects
void onDisconnectedController(ControllerPtr ctl) {
  Serial.println("Controller disconnected!");
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      
      // Send emergency stop when controller disconnects
      sendEmergencyStop();
      break;
    }
  }
}

// Send emergency stop commands - combined command format
void sendEmergencyStop() {
  Serial.println("Sending emergency stop");
  
  // Send once with combined format
  SerialMega.print("LR=0,0\n");
  
  // Send traditional format as backup
  SerialMega.print("L=0\n");
  SerialMega.print("R=0\n");
  SerialMega.print("S=0\n");
}

// Simple helper to clamp values
int clampValue(int v, int minV, int maxV) {
  if (v > maxV) return maxV;
  if (v < minV) return minV;
  return v;
}

// Apply deadzone to any value
int applyDeadzone(int value, int deadzone) {
  if (abs(value) < deadzone) {
    return 0;
  }
  return value;
}

// Send motor commands with rate limiting
void sendMotorCommands(int leftVal, int rightVal) {
  unsigned long currentTime = millis();
  
  // Only send if values changed or enough time has passed
  bool valuesChanged = (leftVal != prevLeftVal || rightVal != prevRightVal);
  bool intervalElapsed = (currentTime - lastCommandTime) >= MIN_COMMAND_INTERVAL_MS;
  bool forceUpdate = (currentTime - lastCommandTime) >= FORCE_COMMAND_INTERVAL_MS;
  
  if ((valuesChanged && intervalElapsed) || forceUpdate) {
    // Combined command format - more efficient
    char cmdCombined[32];
    sprintf(cmdCombined, "LR=%d,%d\n", leftVal, rightVal);
    SerialMega.print(cmdCombined);
    
    // Update tracking variables
    prevLeftVal = leftVal;
    prevRightVal = rightVal;
    lastCommandTime = currentTime;
    
    // Debug output
    Serial.printf("Sent: %s", cmdCombined);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("ESP32 PS5 Controller - Optimized Version");

  // Initialize hardware serial to Arduino Mega
  // Increase baud rate for better communication
  SerialMega.begin(115200, SERIAL_8N1, 16, 17);
  delay(50);

  // Setup Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  Serial.println("Setup done. Waiting for controllers...");
  
  // Send initial stop command
  sendEmergencyStop();
  
  // Initialize timing variables
  lastCommandTime = millis();
}

void loop() {
  // Fetch controller updates - no delay if no update
  BP32.update();
  
  // Check if any controller is connected
  bool isConnected = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] != nullptr && myControllers[i]->isConnected()) {
      isConnected = true;
      break;
    }
  }
  
  // Detect controller disconnect
  if (wasConnected && !isConnected) {
    Serial.println("Controller connection lost!");
    sendEmergencyStop();
  }
  wasConnected = isConnected;
  
  // Process each connected controller
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    ControllerPtr ctl = myControllers[i];
    if (ctl && ctl->isConnected()) {
      // Always read the latest values whether hasData() is true or not
      // This ensures we get the most up-to-date controller state
      
      // Read throttle (R2) and brake (L2)
      int throttleRaw = ctl->throttle(); // R2
      int brakeRaw = ctl->brake();      // L2

      // Read left stick X for steering
      int16_t axisX = ctl->axisX();
      
      // Apply deadzones
      throttleRaw = applyDeadzone(throttleRaw, THROTTLE_DEADZONE);
      brakeRaw = applyDeadzone(brakeRaw, THROTTLE_DEADZONE);
      axisX = applyDeadzone(axisX, STEERING_DEADZONE);

      // Convert to motor speed ranges
      int netSpeed = map(throttleRaw - brakeRaw, -1023, 1023, -1000, 1000);
      int steeringVal = map(axisX, -512, 511, -1000, 1000);

      // Arcade mixing
      int leftVal = netSpeed + steeringVal;
      int rightVal = netSpeed - steeringVal;

      // Clamp speeds
      leftVal = clampValue(leftVal, -1000, 1000);
      rightVal = clampValue(rightVal, -1000, 1000);

      // Send commands (with rate limiting)
      sendMotorCommands(leftVal, rightVal);
    }
  }
  
  // Very short delay for task scheduling
  // This is important for ESP32 background tasks but won't cause stuttering
  delay(5);
}