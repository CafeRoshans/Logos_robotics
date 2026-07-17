import time
import curses

from servo import Servo
import gpio as gpio

NUM_SERVOS = 4
ANGLE_MIN = 0
ANGLE_MAX = 180
ANGLE_STEP = 1

SERVO4_MIN = 44
SERVO4_MAX = 97

servo = Servo()
angles = [0] * (NUM_SERVOS + 1)
current_servo = 1

LEFT_FWD = gpio.IN1
LEFT_BACK = gpio.IN2
RIGHT_BACK = gpio.IN3
RIGHT_FWD = gpio.IN4

speed = 60
motor_state = "стоп"
move_dir = 0
turn_dir = 0
last_dir_key_time = 0.0
DIR_TIMEOUT = 0.2


def clamp_angle(n: int, a: int) -> int:
    if n == 4:
        if a < SERVO4_MIN:
            return SERVO4_MIN
        if a > SERVO4_MAX:
            return SERVO4_MAX
        return a
    if a < ANGLE_MIN:
        return ANGLE_MIN
    if a > ANGLE_MAX:
        return ANGLE_MAX
    return a


def apply_servo(n: int) -> None:
    servo.set(n, angles[n])
    time.sleep(0.02)


def init_servos() -> None:
    for i in range(1, NUM_SERVOS + 1):
        if i == 4:
            angles[i] = (SERVO4_MIN + SERVO4_MAX) // 2
        else:
            angles[i] = 90
        apply_servo(i)


def update_speed() -> None:
    global speed
    if speed < 0:
        speed = 0
    if speed > 100:
        speed = 100
    gpio.ena_pwm(speed)
    gpio.enb_pwm(speed)


def set_motors(lf: int, lb: int, rf: int, rb: int) -> None:
    gpio.digital_write(LEFT_FWD, bool(lf))
    gpio.digital_write(LEFT_BACK, bool(lb))
    gpio.digital_write(RIGHT_FWD, bool(rf))
    gpio.digital_write(RIGHT_BACK, bool(rb))


def motors_forward() -> None:
    global motor_state
    set_motors(1, 0, 1, 0)
    motor_state = "вперёд"


def motors_back() -> None:
    global motor_state
    set_motors(0, 1, 0, 1)
    motor_state = "назад"


def motors_left() -> None:
    global motor_state
    set_motors(0, 1, 1, 0)
    motor_state = "влево"


def motors_right() -> None:
    global motor_state
    set_motors(1, 0, 0, 1)
    motor_state = "вправо"


def motors_arc_forward_right() -> None:
    global motor_state
    set_motors(1, 0, 0, 0)
    motor_state = "дуга вперёд вправо"


def motors_arc_forward_left() -> None:
    global motor_state
    set_motors(0, 0, 1, 0)
    motor_state = "дуга вперёд влево"


def motors_arc_back_right() -> None:
    global motor_state
    set_motors(0, 1, 0, 0)
    motor_state = "дуга назад вправо"


def motors_arc_back_left() -> None:
    global motor_state
    set_motors(0, 0, 0, 1)
    motor_state = "дуга назад влево"


def motors_stop() -> None:
    global motor_state
    set_motors(0, 0, 0, 0)
    motor_state = "стоп"


def update_motors_from_state() -> None:
    if move_dir == 0 and turn_dir == 0:
        motors_stop()
    elif move_dir == 1 and turn_dir == 0:
        motors_forward()
    elif move_dir == -1 and turn_dir == 0:
        motors_back()
    elif move_dir == 0 and turn_dir == -1:
        motors_left()
    elif move_dir == 0 and turn_dir == 1:
        motors_right()
    elif move_dir == 1 and turn_dir == 1:
        motors_arc_forward_right()
    elif move_dir == 1 and turn_dir == -1:
        motors_arc_forward_left()
    elif move_dir == -1 and turn_dir == 1:
        motors_arc_back_right()
    elif move_dir == -1 and turn_dir == -1:
        motors_arc_back_left()


def main(stdscr) -> None:
    global current_servo, ANGLE_STEP, speed, move_dir, turn_dir, last_dir_key_time

    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    init_servos()
    update_speed()

    while True:
        now = time.time()

        ch = stdscr.getch()

        if ch != -1:
            if ch in (ord("q"), ord("Q")):
                break

            if ord("1") <= ch <= ord(str(NUM_SERVOS)):
                current_servo = int(chr(ch))

            elif ch in (ord("+"), ord("=")):
                if ANGLE_STEP < 30:
                    ANGLE_STEP += 1

            elif ch in (ord("-"), ord("_")):
                if ANGLE_STEP > 1:
                    ANGLE_STEP -= 1

            elif ch == ord(","):
                speed -= 5
                update_speed()

            elif ch == ord("."):
                speed += 5
                update_speed()

            elif ch == curses.KEY_UP:
                angles[current_servo] = clamp_angle(current_servo, angles[current_servo] + ANGLE_STEP)
                apply_servo(current_servo)

            elif ch == curses.KEY_DOWN:
                angles[current_servo] = clamp_angle(current_servo, angles[current_servo] - ANGLE_STEP)
                apply_servo(current_servo)

            elif ch in (ord("w"), ord("W")):
                move_dir = 1
                last_dir_key_time = now
                update_motors_from_state()

            elif ch in (ord("s"), ord("S")):
                move_dir = -1
                last_dir_key_time = now
                update_motors_from_state()

            elif ch in (ord("a"), ord("A")):
                turn_dir = -1
                last_dir_key_time = now
                update_motors_from_state()

            elif ch in (ord("d"), ord("D")):
                turn_dir = 1
                last_dir_key_time = now
                update_motors_from_state()

            elif ch in (ord(" "), ord("x"), ord("X")):
                move_dir = 0
                turn_dir = 0
                last_dir_key_time = 0.0
                update_motors_from_state()

        if (move_dir != 0 or turn_dir != 0) and last_dir_key_time > 0.0:
            if now - last_dir_key_time > DIR_TIMEOUT:
                move_dir = 0
                turn_dir = 0
                last_dir_key_time = 0.0
                update_motors_from_state()

        stdscr.clear()
        stdscr.addstr(0, 0, "Пульт робота (control7)")
        stdscr.addstr(1, 0, "1–4: выбор сервы | ↑/↓: угол | +/-: шаг | q: выход")
        stdscr.addstr(2, 0, "WASD: движение, остановка при отпускании | пробел/X: принудительный стоп")
        stdscr.addstr(3, 0, " , / . : скорость")
        stdscr.addstr(4, 0, f"Серво S{current_servo}: {angles[current_servo]}°  шаг: {ANGLE_STEP}°")
        stdscr.addstr(5, 0, f"Моторы: {motor_state}  скорость: {speed}%")

        stdscr.addstr(7, 0, "Углы всех серв:")
        for i in range(1, NUM_SERVOS + 1):
            mark = ">" if i == current_servo else " "
            stdscr.addstr(8 + i, 0, f"{mark} S{i}: {angles[i]}°")

        stdscr.refresh()
        time.sleep(0.01)


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    finally:
        motors_stop()
