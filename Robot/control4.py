#!/usr/bin/env python3
import time
import curses

from servo import Servo
import gpio as gpio   # их GPIO-обёртка, тут и ENA/ENB, и IN1..IN4

# ===== сервоприводы =====
NUM_SERVOS = 8
ANGLE_MIN = 0
ANGLE_MAX = 180
ANGLE_STEP = 1   # базовый шаг по стрелкам

servo = Servo()
angles = [0] * (NUM_SERVOS + 1)   # 1..8
current_servo = 1

# ===== моторы =====
# берём их константы, но жёстко маппим как ты сказал
LEFT_FWD   = gpio.IN1   # 16
LEFT_BACK  = gpio.IN2   # 19
RIGHT_BACK = gpio.IN3   # 26
RIGHT_FWD  = gpio.IN4   # 21

speed = 60              # стартовая скорость (0–100)
motor_state = "стоп"


def clamp_angle(a: int) -> int:
    if a < ANGLE_MIN:
        return ANGLE_MIN
    if a > ANGLE_MAX:
        return ANGLE_MAX
    return a


def apply_servo(n: int):
    servo.set(n, angles[n])
    time.sleep(0.02)


def init_servos():
    for i in range(1, NUM_SERVOS + 1):
        angles[i] = 90
        apply_servo(i)


def update_speed():
    """Обновить PWM на обоих моторах."""
    global speed
    if speed < 0:
        speed = 0
    if speed > 100:
        speed = 100
    gpio.ena_pwm(speed)
    gpio.enb_pwm(speed)


def set_motors(lf: int, lb: int, rf: int, rb: int):
    """lf/lb/rf/rb = 0 или 1: вперёд/назад/стоп."""
    gpio.digital_write(LEFT_FWD,   True if lf else False)
    gpio.digital_write(LEFT_BACK,  True if lb else False)
    gpio.digital_write(RIGHT_FWD,  True if rf else False)
    gpio.digital_write(RIGHT_BACK, True if rb else False)


def motors_forward():
    global motor_state
    set_motors(1, 0, 1, 0)
    motor_state = "вперёд"


def motors_back():
    global motor_state
    set_motors(0, 1, 0, 1)
    motor_state = "назад"


def motors_left():
    """Левый назад, правый вперёд – разворот на месте влево."""
    global motor_state
    set_motors(0, 1, 1, 0)
    motor_state = "влево"


def motors_right():
    """Левый вперёд, правый назад – разворот на месте вправо."""
    global motor_state
    set_motors(1, 0, 0, 1)
    motor_state = "вправо"


def motors_stop():
    global motor_state
    set_motors(0, 0, 0, 0)
    motor_state = "стоп"


def main(stdscr):
    global current_servo, ANGLE_STEP, speed

    curses.curs_set(0)
    stdscr.nodelay(False)
    stdscr.keypad(True)

    init_servos()
    update_speed()

    while True:
        stdscr.clear()
        stdscr.addstr(0, 0, "Пульт робота")
        stdscr.addstr(1, 0,
            "1–8: выбор сервы | ↑/↓: угол | +/-: шаг | q: выход")
        stdscr.addstr(2, 0,
            "WASD: движение | пробел/X: стоп | , / . : скорость")
        stdscr.addstr(3, 0,
            f"Серво S{current_servo}: {angles[current_servo]}°  шаг: {ANGLE_STEP}°")
        stdscr.addstr(4, 0,
            f"Моторы: {motor_state}  скорость: {speed}%")

        stdscr.addstr(6, 0, "Углы всех серв:")
        for i in range(1, NUM_SERVOS + 1):
            mark = ">" if i == current_servo else " "
            stdscr.addstr(7 + i, 0, f"{mark} S{i}: {angles[i]}°")

        stdscr.refresh()
        ch = stdscr.getch()

        # выход
        if ch in (ord('q'), ord('Q')):
            break

        # выбор сервы
        if ord('1') <= ch <= ord(str(NUM_SERVOS)):
            current_servo = int(chr(ch))
            continue

        # изменение шага для серв
        if ch in (ord('+'), ord('=')):
            if ANGLE_STEP < 30:
                ANGLE_STEP += 1
            continue
        if ch in (ord('-'), ord('_')):
            if ANGLE_STEP > 1:
                ANGLE_STEP -= 1
            continue

        # изменение скорости , и .
        if ch == ord(','):
            speed -= 5
            update_speed()
            continue
        if ch == ord('.'):
            speed += 5
            update_speed()
            continue

        # стрелки – сервы
        if ch == curses.KEY_UP:
            angles[current_servo] = clamp_angle(angles[current_servo] + ANGLE_STEP)
            apply_servo(current_servo)
            continue
        if ch == curses.KEY_DOWN:
            angles[current_servo] = clamp_angle(angles[current_servo] - ANGLE_STEP)
            apply_servo(current_servo)
            continue

        # WASD – движение
        if ch in (ord('w'), ord('W')):
            motors_forward()
        elif ch in (ord('s'), ord('S')):
            motors_back()
        elif ch in (ord('a'), ord('A')):
            motors_left()
        elif ch in (ord('d'), ord('D')):
            motors_right()
        elif ch in (ord(' '), ord('x'), ord('X')):
            motors_stop()


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    finally:
        motors_stop()
