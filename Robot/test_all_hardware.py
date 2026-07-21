#!/usr/bin/env python3
import sys
import time

sys.path.append('/root/XiaoRGeek')

try:
    import xr_gpio as gpio
    print("[OK] xr_gpio загружен")
except Exception as e:
    print(f"[FAIL] xr_gpio: {e}")
    sys.exit(1)

try:
    from xr_ultrasonic import Ultrasonic
    us = Ultrasonic()
    print("[OK] Ultrasonic загружен")
except Exception as e:
    print(f"[FAIL] Ultrasonic: {e}")
    us = None

import config as cfg

MOTOR_TEST_DURATION = 1.5
MOTOR_TEST_PWM = 50


def stop_motors():
    gpio.digital_write(gpio.IN1, 0)
    gpio.digital_write(gpio.IN2, 0)
    gpio.digital_write(gpio.IN3, 0)
    gpio.digital_write(gpio.IN4, 0)
    gpio.ena_pwm(0)
    gpio.enb_pwm(0)


def set_motor_left(pwm):
    if cfg.MOTOR_INVERT_LEFT:
        pwm = -pwm
    abs_pwm = abs(pwm)
    if abs_pwm < cfg.MOTOR_DEAD_ZONE:
        abs_pwm = 0
    elif abs_pwm < cfg.MIN_MOTOR_PWM:
        abs_pwm = cfg.MIN_MOTOR_PWM

    gpio.ena_pwm(int(abs_pwm))
    if pwm > 0:
        gpio.digital_write(gpio.IN1, 1)
        gpio.digital_write(gpio.IN2, 0)
    elif pwm < 0:
        gpio.digital_write(gpio.IN1, 0)
        gpio.digital_write(gpio.IN2, 1)
    else:
        gpio.digital_write(gpio.IN1, 0)
        gpio.digital_write(gpio.IN2, 0)


def set_motor_right(pwm):
    if cfg.MOTOR_INVERT_RIGHT:
        pwm = -pwm
    abs_pwm = abs(pwm)
    if abs_pwm < cfg.MOTOR_DEAD_ZONE:
        abs_pwm = 0
    elif abs_pwm < cfg.MIN_MOTOR_PWM:
        abs_pwm = cfg.MIN_MOTOR_PWM
    abs_pwm = min(int(abs_pwm * cfg.RIGHT_MOTOR_BOOST), 100)

    gpio.enb_pwm(int(abs_pwm))
    if pwm > 0:
        gpio.digital_write(gpio.IN3, 1)
        gpio.digital_write(gpio.IN4, 0)
    elif pwm < 0:
        gpio.digital_write(gpio.IN3, 0)
        gpio.digital_write(gpio.IN4, 1)
    else:
        gpio.digital_write(gpio.IN3, 0)
        gpio.digital_write(gpio.IN4, 0)


def drive(left_pwm, right_pwm, duration, label=""):
    if label:
        print(f"  >> {label}  (L={left_pwm:+d}, R={right_pwm:+d}, {duration:.1f}s)")
    set_motor_left(left_pwm)
    set_motor_right(right_pwm)
    time.sleep(duration)
    stop_motors()


# ==========================================
# ДАТЧИКИ
# ==========================================
def read_all_sensors():
    ir_l = 1 if gpio.digital_read(cfg.PIN_IR_LEFT) == 0 else 0
    ir_r = 1 if gpio.digital_read(cfg.PIN_IR_RIGHT) == 0 else 0
    ir_grip = 1 if gpio.digital_read(cfg.PIN_IR_GRIPPER) == 0 else 0

    uz_cm = 0.0
    if us is not None:
        try:
            uz_cm = us.get_distance()
            if uz_cm <= 0 or uz_cm > 500:
                uz_cm = 999.0
        except:
            uz_cm = -1.0

    return {
        'ir_left': ir_l,
        'ir_right': ir_r,
        'ir_gripper': ir_grip,
        'uz_cm': uz_cm,
    }


def sensor_monitor():
    print("\n=== МОНИТОРИНГ ДАТЧИКОВ (Ctrl+C для выхода) ===")
    print(f"  ИК лев (pin {cfg.PIN_IR_LEFT}) | ИК прав (pin {cfg.PIN_IR_RIGHT}) | ИК клешня (pin {cfg.PIN_IR_GRIPPER}) | УЗ (TRIG={cfg.TRIG},ECHO={cfg.ECHO})")
    print("  0=свободно, 1=объект рядом")
    print()

    try:
        while True:
            s = read_all_sensors()

            ir_l_str = "ОБЪЕКТ" if s['ir_left'] else "------"
            ir_r_str = "ОБЪЕКТ" if s['ir_right'] else "------"
            ir_g_str = "ОБЪЕКТ" if s['ir_gripper'] else "------"
            uz_str = f"{s['uz_cm']:6.1f}cm" if s['uz_cm'] >= 0 else " ERROR"

            sys.stdout.write(
                f"\r  Лев: {s['ir_left']} [{ir_l_str}] | "
                f"Прав: {s['ir_right']} [{ir_r_str}] | "
                f"Клешня: {s['ir_gripper']} [{ir_g_str}] | "
                f"УЗ: {uz_str}   "
            )
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n")


# ==========================================
# МОТОРНЫЕ ТЕСТЫ
# ==========================================
def test_forward():
    print("\n--- Тест: ВПЕРЁД ---")
    drive(MOTOR_TEST_PWM, MOTOR_TEST_PWM, MOTOR_TEST_DURATION, "Обе гусеницы вперёд")
    print("  Готово.\n")


def test_backward():
    print("\n--- Тест: НАЗАД ---")
    drive(-MOTOR_TEST_PWM, -MOTOR_TEST_PWM, MOTOR_TEST_DURATION, "Обе гусеницы назад")
    print("  Готово.\n")


def test_spin_left():
    print("\n--- Тест: РАЗВОРОТ ВЛЕВО (на месте) ---")
    drive(-MOTOR_TEST_PWM, MOTOR_TEST_PWM, MOTOR_TEST_DURATION, "Левая назад, правая вперёд")
    print("  Готово.\n")


def test_spin_right():
    print("\n--- Тест: РАЗВОРОТ ВПРАВО (на месте) ---")
    drive(MOTOR_TEST_PWM, -MOTOR_TEST_PWM, MOTOR_TEST_DURATION, "Левая вперёд, правая назад")
    print("  Готово.\n")


def test_arc_left():
    print("\n--- Тест: ПЛАВНЫЙ ПОВОРОТ ВЛЕВО (дуга) ---")
    drive(int(MOTOR_TEST_PWM * 0.3), MOTOR_TEST_PWM, MOTOR_TEST_DURATION, "Левая 30%, правая 100%")
    print("  Готово.\n")


def test_arc_right():
    print("\n--- Тест: ПЛАВНЫЙ ПОВОРОТ ВПРАВО (дуга) ---")
    drive(MOTOR_TEST_PWM, int(MOTOR_TEST_PWM * 0.3), MOTOR_TEST_DURATION, "Левая 100%, правая 30%")
    print("  Готово.\n")


def test_pwm_ramp():
    print("\n--- Тест: PWM-РАМПА (разгон → торможение) ---")
    print("  Плавный разгон 0 → 100% → 0 за 3 секунды")

    steps = 20
    for i in range(steps + 1):
        pwm = int(100 * i / steps)
        set_motor_left(pwm)
        set_motor_right(pwm)
        sys.stdout.write(f"\r  Разгон: PWM = {pwm:3d}%")
        sys.stdout.flush()
        time.sleep(1.5 / steps)

    for i in range(steps, -1, -1):
        pwm = int(100 * i / steps)
        set_motor_left(pwm)
        set_motor_right(pwm)
        sys.stdout.write(f"\r  Торможение: PWM = {pwm:3d}%")
        sys.stdout.flush()
        time.sleep(1.5 / steps)

    stop_motors()
    print("\n  Готово.\n")


def test_all_motors():
    print("\n===== ПОЛНЫЙ ПРОГОН МОТОРОВ =====")
    print("  Робот выполнит все маневры последовательно.")
    print("  Убедись что робот на полу с запасом пространства!")
    input("  Нажми Enter для старта...")

    test_forward()
    time.sleep(0.5)
    test_backward()
    time.sleep(0.5)
    test_spin_left()
    time.sleep(0.5)
    test_spin_right()
    time.sleep(0.5)
    test_arc_left()
    time.sleep(0.5)
    test_arc_right()
    time.sleep(0.5)
    test_pwm_ramp()

    print("===== ВСЕ ТЕСТЫ ЗАВЕРШЕНЫ =====\n")


# ==========================================
# МЕНЮ
# ==========================================
def main():
    print("\n" + "=" * 50)
    print("  ПОЛНАЯ ПРОВЕРКА ЖЕЛЕЗА GFS-X")
    print("=" * 50)

    while True:
        print("\n  1. Мониторинг датчиков (ИК x3 + УЗ)")
        print("  2. Моторы: вперёд")
        print("  3. Моторы: назад")
        print("  4. Моторы: разворот влево")
        print("  5. Моторы: разворот вправо")
        print("  6. Моторы: дуга влево")
        print("  7. Моторы: дуга вправо")
        print("  8. Моторы: PWM-рампа (разгон/торможение)")
        print("  9. Полный прогон всех моторных тестов")
        print("  0. Выход")

        try:
            choice = input("\n  Выбери пункт: ").strip()
        except (EOFError, KeyboardInterrupt):
            choice = '0'

        if choice == '1':
            sensor_monitor()
        elif choice == '2':
            test_forward()
        elif choice == '3':
            test_backward()
        elif choice == '4':
            test_spin_left()
        elif choice == '5':
            test_spin_right()
        elif choice == '6':
            test_arc_left()
        elif choice == '7':
            test_arc_right()
        elif choice == '8':
            test_pwm_ramp()
        elif choice == '9':
            test_all_motors()
        elif choice == '0':
            break
        else:
            print("  Неизвестный пункт.")

    stop_motors()
    print("Выход. Моторы остановлены.\n")


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        stop_motors()
        print(f"\n[ОШИБКА] {e}")
        raise
    finally:
        stop_motors()
