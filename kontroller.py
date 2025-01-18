#!/usr/bin/env python3

import pygame
import serial
import time
import sys

def main():
    # === 1) Init Pygame ===
    pygame.init()
    pygame.joystick.init()
    joy_count = pygame.joystick.get_count()
    print(f"Pygame sees {joy_count} joysticks.", flush=True)
    joystick = None
    if joy_count > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Using joystick: {joystick.get_name()}", flush=True)
    else:
        print("No joystick connected. Exiting.")
        sys.exit(1)

    # === 2) Init Serial ===
    try:
        ser = serial.Serial('COM3', 115200, timeout=0.1)
    except Exception as e:
        print(f"Could not open serial port: {e}")
        sys.exit(1)

    time.sleep(2)  # one-time wait to let Arduino reset
    print("Starting R2/L2 + Steering control loop...", flush=True)

    # -- Axis indices (likely for PS4/PS5 on many systems) --
    # Double-check with a quick axis print if needed
    AXIS_L2 = 4      # L2 trigger
    AXIS_R2 = 5      # R2 trigger
    AXIS_STEER = 0   # Left stick X for steering

    def convertTrigger(val):
        """
        Convert trigger from [-1..+1] => [0..1].
          - -1 means not pressed
          - +1 means fully pressed
        """
        return (val + 1.0) / 2.0

    def deadzone(value, threshold=0.05):
        return 0.0 if abs(value) < threshold else value

    loop_count = 0
    while True:
        pygame.event.pump()
        loop_count += 1

        # If joystick is present, read the triggers + steering axis
        raw_l2 = joystick.get_axis(AXIS_L2)   # typically -1..+1
        raw_r2 = joystick.get_axis(AXIS_R2)
        raw_steer = joystick.get_axis(AXIS_STEER)  # left stick X, typically -1..+1

        # Convert L2/R2 to 0..1 range
        l2_val = convertTrigger(raw_l2)  # 0=unpressed, 1=fully pressed
        r2_val = convertTrigger(raw_r2)

        # Apply a small deadzone to steering
        steer = deadzone(raw_steer, 0.05)

        # Scale triggers to 0..1000
        throttle_speed = int(r2_val * 1000)  # 0..1000
        reverse_speed  = int(l2_val * 1000)  # 0..1000
        steering_val   = int(steer * 1000)   # -1000..+1000

        # Net speed: R2 for forward, L2 for reverse => net_speed in [-1000..+1000]
        net_speed = throttle_speed - reverse_speed

        # Arcade mixing for left & right motors:
        #   left_motor  = net_speed + steering
        #   right_motor = net_speed - steering
        left_val  = net_speed + steering_val
        right_val = net_speed - steering_val

        # Clamp to -1000..+1000
        left_val  = max(-1000, min(1000, left_val))
        right_val = max(-1000, min(1000, right_val))

        # Print some debug info
        print(
            f"Loop={loop_count}, "
            f"L2={l2_val:.2f}, R2={r2_val:.2f}, Steer={steer:.2f}, "
            f"LeftVal={left_val}, RightVal={right_val}",
            flush=True
        )


        # Send commands to Arduino
        cmd_left = f"L={left_val}\n"
        cmd_right = f"R={right_val}\n"
        ser.write(cmd_left.encode('utf-8'))
        ser.write(cmd_right.encode('utf-8'))

        # Read any Arduino response
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print(f"Arduino replied: {response}", flush=True)

        # ~40 updates/second
        time.sleep(0.1)

if __name__ == "__main__":
    main()
