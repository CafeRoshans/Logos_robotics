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
EMA_STEER = 0.95 # предлагается учить почти без стира 

SERVO_BASE = 1
SERVO_SHOULDER = 2
SERVO_ELBOW = 3
SERVO_CLAW = 4
SERVO_CAMERA_PAN = 8
SERVO_CAMERA_LOW = 7

ANGLE_BASE_CENTER = 87
ANGLE_SHOULDER_UP = 180
ANGLE_ELBOW_UP = 90
ANGLE_SHOULDER_DOWN = 180
ANGLE_ELBOW_DOWN = 90
ANGLE_CLAW_OPEN = 30
ANGLE_CLAW_CLOSE = 70
SERVO_CAMERA_ANGLE = 100

MAX_CAMERA_STEP = 8

# Макс. угол поворота камеры/УЗ от центра (град). ДОЛЖЕН совпадать с
# camera_servo_max_angle в конфигах обучения (Unity). УЗ жёстко связан с
# камерой: при большом угле УЗ не видит препятствия по курсу. ±20° = камера
# почти всегда вперёд, мяч центрируется доворотом корпуса.
CAMERA_SERVO_MAX_ANGLE = 20.0
CAMERA_SERVO_CENTER = 90.0  # механический центр серво (град)

# Авто-центрирование камеры когда мяч не виден: УЗ смотрит вперёд.
AUTO_CENTER_CAMERA = True

# ==========================================
# БЕЗОПАСНОСТЬ
# ==========================================
WATCHDOG_TIMEOUT = 0.5
SAFETY_STOP_CM = 50
