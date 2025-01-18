#!/usr/bin/env python3

import pygame
import serial
import time
import sys

def main():
    # 1) Initialize Pygame for joystick input
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() < 1:
        print("No joystick found!")
        sys.exit(1)

    # Assuming only 1 joystick; if multiple, pick the right index
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Detected controller: {joystick.get_name()}")

    # 2) Initialize serial to Arduino
    #    Change 'COM5' to whatever your Arduino is on (Windows, Mac, or Linux).
    try:
        ser = serial.Serial('COM3', 115200, timeout=0.1)
    except Exception as e:
        print(f"Could not open serial port: {e}")
        sys.exit(1)

    time.sleep(2)  # Give Arduino time to reset if needed

    print("Starting main loop. Press CTRL+C to quit.")

    try:
        while True:
            # Process Pygame events so it can update joystick states
            pygame.event.pump()

            # read the left stick Y axis for forward/back
            #   NOTE: on many PS controllers, axis(1) is the left stick Y,
            #   with -1 at top, +1 at bottom. So we invert it to make
            #   forward = positive numbers.
            left_stick_y = -joystick.get_axis(1)  # invert

            # read the right stick X axis for turning
            #   This might be axis(2) or axis(3) depending on the controller.
            #   Many PS controllers have axis(2) as L2 analog, and axis(3) as right stick X.
            right_stick_x = joystick.get_axis(2)  # Check this if you see no turning

            # Convert from -1..+1 range to -1000..+1000
            forward_speed = int(left_stick_y * 1000)
            turn_speed    = int(right_stick_x * 1000)

            # Simple "arcade mixing":
            #   left_motor  = forward + turn
            #   right_motor = forward - turn
            left_motor_speed  = forward_speed + turn_speed
            right_motor_speed = forward_speed - turn_speed

            # Clamp to -1000..+1000
            if left_motor_speed > 1000:
                left_motor_speed = 1000
            elif left_motor_speed < -1000:
                left_motor_speed = -1000

            if right_motor_speed > 1000:
                right_motor_speed = 1000
            elif right_motor_speed < -1000:
                right_motor_speed = -1000

            # Send to Arduino (e.g. L=NNN  R=NNN)
            # End with newline so Arduino's Serial.readStringUntil('\n') catches it
            cmd_left = f"L={left_motor_speed}\n"
            cmd_right = f"R={right_motor_speed}\n"

            ser.write(cmd_left.encode('utf-8'))
            ser.write(cmd_right.encode('utf-8'))

            # small delay
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()
        pygame.quit()

if __name__ == "__main__":
    main()
