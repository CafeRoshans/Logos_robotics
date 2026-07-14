#!/usr/bin/env python3
"""
test_gripper_ir.py — Автономный тест клешни по ИК датчику
=========================================================
Запускать НАПРЯМУЮ на Raspberry Pi (без ROS и без Unity).

Действие:
  - Перекрой ИК датчик в клешне рукой → клешня ЗАКРОЕТСЯ
  - Убери руку             → клешня ОТКРОЕТСЯ

Пин: IR_M = GPIO 22 (IRM на плате расширения)

Запуск:
  python3 /root/test_gripper_ir.py
"""

import sys
import time

sys.path.append('/root/XiaoRGeek')

# --- Загрузка GPIO ---
try:
    import xr_gpio as gpio
    print("✅ xr_gpio загружен (GPIO пины инициализированы)")
except Exception as e:
    print(f"❌ Ошибка загрузки xr_gpio: {e}")
    sys.exit(1)

# --- Загрузка серво ---
try:
    from xr_servo import Servo
    servo = Servo()
    print("✅ xr_servo загружен")
except Exception as e:
    print(f"❌ Ошибка загрузки xr_servo: {e}")
    sys.exit(1)

# -------------------------------------------------------
# КАЛИБРОВКА КЛЕШНИ — подстрой под свою физическую клешню
# -------------------------------------------------------
SERVO_CLAW      = 4    # Номер сервопривода клешни
SERVO_SHOULDER  = 2    # Плечо
SERVO_ELBOW     = 3    # Локоть

ANGLE_CLAW_OPEN  = 50  # Открыта (как в unity_master.py)
ANGLE_CLAW_CLOSE = 89  # Закрыта (захват)

ANGLE_SHOULDER_DOWN = 20   # Рабочее положение плеча
ANGLE_ELBOW_DOWN    = 130  # Рабочее положение локтя

# Фильтр дребезга: сколько подряд == 0 нужно для срабатывания
DEBOUNCE_COUNT = 3
POLL_HZ = 20           # Частота опроса датчика (раз/сек)
# -------------------------------------------------------

def init_arm_down():
    """Опускаем руку в рабочее положение для теста."""
    print("🦾 Опускаем руку в рабочее положение...")
    servo.set(SERVO_SHOULDER, ANGLE_SHOULDER_DOWN)
    time.sleep(0.3)
    servo.set(SERVO_ELBOW, ANGLE_ELBOW_DOWN)
    time.sleep(0.3)
    servo.set(SERVO_CLAW, ANGLE_CLAW_OPEN)
    time.sleep(0.3)
    print(f"✅ Рука опущена, клешня открыта ({ANGLE_CLAW_OPEN}°)")

def open_claw():
    servo.set(SERVO_CLAW, ANGLE_CLAW_OPEN)
    print(f"  ↕  Клешня ОТКРЫТА ({ANGLE_CLAW_OPEN}°)")

def close_claw():
    servo.set(SERVO_CLAW, ANGLE_CLAW_CLOSE)
    print(f"  ✊ Клешня ЗАКРЫТА ({ANGLE_CLAW_CLOSE}°)  ← мяч схвачен!")

def read_ir_m():
    """
    Читаем IR_M (pin 22).
    Датчик с pull-up: 0 = препятствие есть, 1 = пусто.
    Возвращаем 1 если препятствие (инвертируем для удобства).
    """
    return 1 if gpio.digital_read(gpio.IR_M) == 0 else 0

# === MAIN ===
print("\n========================================")
print("   ТЕСТ КЛЕШНИ — ИК ДАТЧИК IR_M (pin 22)")
print("========================================")
print("Перекрой датчик в клешне → клешня ЗАКРОЕТСЯ")
print("Убери руку               → клешня ОТКРОЕТСЯ")
print("Ctrl+C для выхода\n")

init_arm_down()

debounce_buf = [0] * DEBOUNCE_COUNT
claw_closed = False
interval = 1.0 / POLL_HZ

try:
    while True:
        raw = read_ir_m()

        # Сдвигаем буфер дребезга
        debounce_buf.pop(0)
        debounce_buf.append(raw)

        # Стабильное срабатывание = все значения в буфере == raw
        if all(v == 1 for v in debounce_buf):  # стабильно видим объект
            if not claw_closed:
                close_claw()
                claw_closed = True
        elif all(v == 0 for v in debounce_buf):  # стабильно пусто
            if claw_closed:
                open_claw()
                claw_closed = False

        # Статус в строке
        status = "ОБЪЕКТ" if raw == 1 else "пусто "
        claw_str = "ЗАКРЫТА" if claw_closed else "открыта"
        sys.stdout.write(f"\r  IR_M(22): {raw} [{status}]  |  Клешня: {claw_str}   ")
        sys.stdout.flush()

        time.sleep(interval)

except KeyboardInterrupt:
    print("\n\n🛑 Тест остановлен. Убираем руку в стартовое положение...")
    open_claw()
    time.sleep(0.3)
    print("✅ Готово.")
