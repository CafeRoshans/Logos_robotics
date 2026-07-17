import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Все пины, кроме тех, что управляют моторами (чтобы робот не уехал)
safe_pins = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15, 18, 22, 23, 24, 25, 27]
pin_states = {}

for p in safe_pins:
    try:
        GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        pin_states[p] = GPIO.input(p)
    except:
        pass

print("Радар запущен! Вставьте датчик в порт IO_3.")
print("Помашите рукой перед датчиком. Скрипт покажет, какой пин на Малине сработал!")
print("Для выхода нажмите Ctrl+C\n")

try:
    while True:
        for p in list(pin_states.keys()):
            val = GPIO.input(p)
            if val != pin_states[p]:
                print(f"--> Бинго! Сработал GPIO пин номер: {p} (значение: {val})")
                pin_states[p] = val
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nРадар остановлен.")
