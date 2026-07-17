#!/usr/bin/env python3
import sys
import time
sys.path.append('/root/XiaoRGeek')
from xr_servo import Servo

servo = Servo()
SERVO_CLAW = 4

print("--- XiaoR Geek Claw Calibration Tool ---")
print("This tool will move the claw slowly. If you hear grinding, note the angle and STOP.")

try:
    current_angle = 90
    print(f"Moving to middle position (90)...")
    servo.set(SERVO_CLAW, 90)
    time.sleep(1)
    
    while True:
        target = input("Enter target angle (15-160) or 'q' to quit: ")
        if target == 'q': break
        try:
            angle = int(target)
            print(f"Moving to {angle}...")
            servo.set(SERVO_CLAW, angle)
        except ValueError:
            print("Invalid input.")
except KeyboardInterrupt:
    pass
print("Done.")
