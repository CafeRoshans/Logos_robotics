#!/usr/bin/env python3

# ir sensors, got via detect_pin.py
PIN_IR_LEFT = 23
PIN_IR_RIGHT = 24
PIN_IR_GRIPPER = 25

TRIG = 17 # send wave
ECHO = 4 # get wave

MOTOR_INVERT_LEFT = False
MOTOR_INVERT_RIGHT = False
RIGHT_MOTOR_BOOST = 1.07

MIN_MOTOR_PWM = 35
MOTOR_DEAD_ZONE = 10
MAX_PWM_STEP = 15

L = 0.4
MAX_SPEED_M_S = 0.5
PWM_CONVERSION_FACTOR = 100.0 / MAX_SPEED_M_S

TURN_K = 0.25
MAX_LINEAR = 0.25
EMA_STEER = 0.40

SERVO_BASE = 1
SERVO_SHOULDER = 2
SERVO_ELBOW = 3
SERVO_CLAW = 4
SERVO_CAMERA_PAN = 8
SERVO_CAMERA_LOW = 7  # TODO (не подтверждено, 2026-07-21): похоже на тилт-сервопривод камеры,
                      # но в numpy_brain.py выставляется один раз при инициализации и никогда
                      # не управляется динамически. Нужно физически проверить на роботе, что
                      # это за серво, прежде чем опираться на него в новом коде управления.

ANGLE_BASE_CENTER = 87
ANGLE_SHOULDER_UP = 180
ANGLE_ELBOW_UP = 90
ANGLE_SHOULDER_DOWN = 180
ANGLE_ELBOW_DOWN = 90
ANGLE_CLAW_OPEN = 30
ANGLE_CLAW_CLOSE = 70
SERVO_CAMERA_ANGLE = 100

MAX_CAMERA_STEP = 15

# ==========================================
# БЕЗОПАСНОСТЬ
# ==========================================
WATCHDOG_TIMEOUT = 0.5
SAFETY_STOP_CM = 50
