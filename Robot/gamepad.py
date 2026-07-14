#!/usr/bin/env python3
import os
import struct
import time

from xr_servo import Servo
import xr_gpio as gpio

JS_DEV = "/dev/input/js0"

AXIS_LX = 0
AXIS_LY = 1
AXIS_LT = 2
AXIS_RX = 3
AXIS_RY = 4
AXIS_RT = 5

BTN_STOP = 1
BTN_GRIP_OPEN = 4
BTN_SECOND_UP = 5

DEBUG_EVENTS = False

NUM_SERVOS = 4

SERVO_MAIN = 1
SERVO_ROTATE = 2
SERVO_SECOND = 3
SERVO_GRIPPER = 4

ANGLE_MIN = 0
ANGLE_MAX = 180

GRIP_MIN = 44
GRIP_MAX = 97

ANGLE_SPEED_STICK = 2.0
ANGLE_SPEED_TRIGGER = 2.0
ANGLE_SPEED_BUTTON = 2.0

servo = Servo()
angles = [0] * (NUM_SERVOS + 1)

LEFT_FWD = gpio.IN1
LEFT_BACK = gpio.IN2
RIGHT_BACK = gpio.IN3
RIGHT_FWD = gpio.IN4

base_speed = 60
motor_state = "стоп"

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

EVENT_FORMAT = "IhBB"
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)


def clamp(val, lo, hi):
    return lo if val < lo else hi if val > hi else val


def clamp_angle(a):
    return int(clamp(int(a), ANGLE_MIN, ANGLE_MAX))


def clamp_gripper(a):
    return int(clamp(int(a), GRIP_MIN, GRIP_MAX))


def apply_servo(n):
    servo.set(n, angles[n])
    time.sleep(0.02)


def init_servos():
    for i in range(1, NUM_SERVOS + 1):
        angles[i] = 90
    angles[SERVO_GRIPPER] = (GRIP_MIN + GRIP_MAX) // 2
    for i in range(1, NUM_SERVOS + 1):
        apply_servo(i)


def update_speed(pct):
    spd = int(clamp(pct, 0, 100))
    gpio.ena_pwm(spd)
    gpio.enb_pwm(spd)


def set_motors_raw(left_dir, right_dir, speed_pct):
    global motor_state

    lf = lb = rf = rb = 0

    if left_dir > 0:
        lf = 1
    elif left_dir < 0:
        lb = 1

    if right_dir > 0:
        rf = 1
    elif right_dir < 0:
        rb = 1

    gpio.digital_write(LEFT_FWD, bool(lf))
    gpio.digital_write(LEFT_BACK, bool(lb))
    gpio.digital_write(RIGHT_FWD, bool(rf))
    gpio.digital_write(RIGHT_BACK, bool(rb))

    update_speed(speed_pct)

    if left_dir == 0 and right_dir == 0:
        motor_state = "стоп"
    elif left_dir == right_dir == 1:
        motor_state = "вперёд"
    elif left_dir == right_dir == -1:
        motor_state = "назад"
    elif left_dir > right_dir:
        motor_state = "вправо"
    elif left_dir < right_dir:
        motor_state = "влево"
    else:
        motor_state = "движение"


def motors_stop():
    set_motors_raw(0, 0, 0)


def open_joystick():
    return os.open(JS_DEV, os.O_RDONLY | os.O_NONBLOCK)


def read_events(fd, axes, buttons):
    while True:
        try:
            evbuf = os.read(fd, EVENT_SIZE)
        except BlockingIOError:
            break

        if not evbuf or len(evbuf) < EVENT_SIZE:
            break

        t, val, etype, num = struct.unpack(EVENT_FORMAT, evbuf)

        if etype & JS_EVENT_INIT:
            etype &= ~JS_EVENT_INIT

        if etype == JS_EVENT_AXIS:
            v = val / (32767.0 if val >= 0 else 32768.0)
            v = clamp(v, -1.0, 1.0)
            axes[num] = v
            if DEBUG_EVENTS and abs(v) > 0.2:
                print(f"AXIS {num}: {v:.2f}")

        elif etype == JS_EVENT_BUTTON:
            buttons[num] = (val != 0)
            if DEBUG_EVENTS:
                print(f"BUTTON {num}: {'down' if buttons[num] else 'up'}")


def process_controls(axes, buttons):
    global angles

    ly = axes.get(AXIS_LY, 0.0)
    lx = axes.get(AXIS_LX, 0.0)

    deadzone = 0.2

    move = -ly
    turn = lx

    left_dir = right_dir = 0
    speed_pct = 0.0

    if abs(move) < deadzone:
        move = 0.0
    if abs(turn) < deadzone:
        turn = 0.0

    if move != 0.0 or turn != 0.0:
        left = clamp(move + turn, -1.0, 1.0)
        right = clamp(move - turn, -1.0, 1.0)

        m = max(abs(left), abs(right))
        if m > 0:
            left /= m
            right /= m

        left_dir = 1 if left > 0.1 else -1 if left < -0.1 else 0
        right_dir = 1 if right > 0.1 else -1 if right < -0.1 else 0

        speed_pct = base_speed * max(abs(left), abs(right))

    if buttons.get(BTN_STOP, False):
        motors_stop()
    else:
        set_motors_raw(left_dir, right_dir, speed_pct)

    deadzone_servo = 0.15
    ry = axes.get(AXIS_RY, 0.0)
    rx = axes.get(AXIS_RX, 0.0)

    if abs(ry) >= deadzone_servo:
        new_angle = clamp_angle(angles[SERVO_MAIN] - ry * ANGLE_SPEED_STICK)
        if new_angle != angles[SERVO_MAIN]:
            angles[SERVO_MAIN] = new_angle
            apply_servo(SERVO_MAIN)

    if abs(rx) >= deadzone_servo:
        new_angle = clamp_angle(angles[SERVO_ROTATE] + rx * ANGLE_SPEED_STICK)
        if new_angle != angles[SERVO_ROTATE]:
            angles[SERVO_ROTATE] = new_angle
            apply_servo(SERVO_ROTATE)

    lt = axes.get(AXIS_LT, -1.0)
    rt = axes.get(AXIS_RT, -1.0)

    pressed_thr = -0.7

    if lt > pressed_thr:
        new_angle = clamp_gripper(angles[SERVO_GRIPPER] + ANGLE_SPEED_TRIGGER)
        if new_angle != angles[SERVO_GRIPPER]:
            angles[SERVO_GRIPPER] = new_angle
            apply_servo(SERVO_GRIPPER)

    if buttons.get(BTN_GRIP_OPEN, False):
        new_angle = clamp_gripper(angles[SERVO_GRIPPER] - ANGLE_SPEED_BUTTON)
        if new_angle != angles[SERVO_GRIPPER]:
            angles[SERVO_GRIPPER] = new_angle
            apply_servo(SERVO_GRIPPER)

    if rt > pressed_thr:
        new_angle = clamp_angle(angles[SERVO_SECOND] - ANGLE_SPEED_TRIGGER)
        if new_angle != angles[SERVO_SECOND]:
            angles[SERVO_SECOND] = new_angle
            apply_servo(SERVO_SECOND)

    if buttons.get(BTN_SECOND_UP, False):
        new_angle = clamp_angle(angles[SERVO_SECOND] + ANGLE_SPEED_BUTTON)
        if new_angle != angles[SERVO_SECOND]:
            angles[SERVO_SECOND] = new_angle
            apply_servo(SERVO_SECOND)


def main():
    print("Старт управления роботом с геймпада.")
    print(f"Ожидаю геймпад на {JS_DEV}. Запускай от root, если что.")

    init_servos()
    motors_stop()

    axes = {}
    buttons = {}

    fd = open_joystick()

    try:
        while True:
            read_events(fd, axes, buttons)
            process_controls(axes, buttons)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nОстановка по Ctrl+C")
    finally:
        motors_stop()
        os.close(fd)


if __name__ == "__main__":
    main()
