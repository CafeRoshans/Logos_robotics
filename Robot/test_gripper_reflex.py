import RPi.GPIO as GPIO
import time
import sys

# Добавляем путь к библиотекам XiaoR Geek
sys.path.append('/root/XiaoRGeek')

try:
    from xr_servo import Servo
    servo = Servo()
except Exception as e:
    print(f"Ошибка загрузки драйвера серво: {e}")
    sys.exit()

# Настройки пинов и углов
IR_PIN = 24
CLAW_SERVO_INDEX = 4
ANGLE_OPEN = 50
ANGLE_CLOSE = 89

# Инициализация GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("=== ТЕСТ АВТО-КЛЕШНИ ЗАПУЩЕН ===")
print(f"Используем пин датчика: {IR_PIN}")
print(f"Используем индекс серво: {CLAW_SERVO_INDEX}")
print("Поднесите руку к датчику в клешне...")
print("Для выхода нажмите Ctrl+C")

current_state = -1 # -1: неизвестно, 0: открыто, 1: закрыто

try:
    # При старте открываем
    servo.set(CLAW_SERVO_INDEX, ANGLE_OPEN)
    current_state = 0
    
    while True:
        # Читаем датчик (0 - видит объект, 1 - пусто)
        ir_value = GPIO.input(IR_PIN)
        
        if ir_value == 0 and current_state != 1:
            print("--> ОБЪЕКТ ОБНАРУЖЕН! Закрываю клешню...")
            servo.set(CLAW_SERVO_INDEX, ANGLE_CLOSE)
            current_state = 1
            
        elif ir_value == 1 and current_state != 0:
            print("--> ПУСТО. Открываю клешню...")
            servo.set(CLAW_SERVO_INDEX, ANGLE_OPEN)
            current_state = 0
            
        time.sleep(0.05) # Частота 20 Гц

except KeyboardInterrupt:
    print("\nТест остановлен.")
    # Возвращаем в исходное
    servo.set(CLAW_SERVO_INDEX, ANGLE_OPEN)
finally:
    GPIO.cleanup(IR_PIN)
