/*********************************************************************************
  ESP32 Sketch using Bluepad32 to read PS5 controller, then send L= / R= commands
  to Arduino Mega for your hoverboard car.

  Steps:
  1) Wire ESP32 TX/RX pins to Mega RX1/TX1 (or whichever hardware serial you use).
     - For example:
       ESP32 GPIO 17 (TX2) -> Mega pin 19 (RX1)
       ESP32 GPIO 16 (RX2) -> Mega pin 18 (TX1)
       Common GND
  2) On Mega, read from Serial1 instead of Serial (or adjust wiring to use pins 0/1).
  3) The code here does "arcade mixing":
       netSpeed = throttleSpeed - reverseSpeed
       leftVal  = netSpeed + steeringVal
       rightVal = netSpeed - steeringVal
     which matches your Python script logic.
**********************************************************************************/

#include <Arduino.h>
#include <Bluepad32.h>
#include <HardwareSerial.h>

//
// CREATE A HARDWARE SERIAL INTERFACE FOR MEGA
// We'll pick UART2 on the ESP32. On many dev boards, that uses GPIO 16 (RX2) and 17 (TX2).
// Adjust these pins if your ESP32 board uses different ones.
HardwareSerial SerialMega(2); // "2" = UART2 instance

// Bluepad32 can connect up to 4 controllers.
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Called when a new controller connects
void onConnectedController(ControllerPtr ctl) {
  Serial.println("CALLBACK: A controller just connected!");
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
  Serial.println("CALLBACK: A controller disconnected!");
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      break;
    }
  }
}

// Simple helper to clamp values
int clampValue(int v, int minV, int maxV) {
  if (v > maxV) return maxV;
  if (v < minV) return minV;
  return v;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP32 PS5 Controller with Arcade Mixing - Starting...");

  // 1) Initialize hardware serial to Arduino Mega
  //    115200 baud, 8N1, with RX/TX pins for UART2 = 16 (RX2), 17 (TX2) on many boards
  //    If your board is different, adjust the pins
  SerialMega.begin(115200, SERIAL_8N1, 16, 17);
  delay(100);

  // 2) Setup Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Optionally "forgetBluetoothKeys" if you want to re-pair from scratch each time
  // BP32.forgetBluetoothKeys();

  // Some gamepads might appear as multiple devices (mouse, keyboard).
  // We'll keep it simple for a standard gamepad, so we disable virtual devices:
  BP32.enableVirtualDevice(false);

  Serial.println("Setup done. Waiting for controllers...");
}

void loop() {
  // 1) Fetch controller updates
  bool updated = BP32.update();
  if (!updated) {
    // If nothing changed, just wait a bit
    delay(10);
    return;
  }

  // 2) Process each connected controller
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    ControllerPtr ctl = myControllers[i];
    if (ctl && ctl->isConnected() && ctl->hasData()) {

      // A) Read throttle (R2) and brake (L2) from the Bluepad32 system
      //    Typically 0..1023 for throttle() and brake(), 0 means unpressed, 1023 means fully pressed
      int throttleRaw = ctl->throttle(); // R2
      int brakeRaw    = ctl->brake();    // L2

      // B) Read left stick X for steering, typically -512..+511
      int16_t axisX = ctl->axisX();

      // C) Convert R2 (throttle) / L2 (brake) to -1000..+1000 range using basic "map" logic
      //    netSpeed = throttle - brake, each 0..1023 => net range -1023..+1023 => -1000..+1000
      int netSpeed = map(throttleRaw - brakeRaw, -1023, 1023, -1000, 1000);

      // D) Convert axisX from -512..+511 => -1000..+1000 for steering
      int steeringVal = map(axisX, -512, 511, -1000, 1000);

      // E) Arcade mixing
      int leftVal  = netSpeed + steeringVal;
      int rightVal = netSpeed - steeringVal;

      // F) Clamp final speeds to -1000..+1000
      leftVal  = clampValue(leftVal, -1000, 1000);
      rightVal = clampValue(rightVal, -1000, 1000);

      // G) Send to Arduino Mega as "L=NNN\n" and "R=NNN\n"
      char cmdLeft[16];
      char cmdRight[16];
      sprintf(cmdLeft,  "L=%d\n", leftVal);
      sprintf(cmdRight, "R=%d\n", rightVal);

      SerialMega.print(cmdLeft);
      SerialMega.print(cmdRight);

      // (Optional) Print debugging to USB for us to see
      Serial.printf("Controller %d: throttle=%d, brake=%d, axisX=%d => L=%d, R=%d\n",
                    i, throttleRaw, brakeRaw, axisX, leftVal, rightVal);
    }
  }

  // If you want a specific loop speed, do a small delay
  delay(20);
}
