import xr_gpio as gpio
import time
import sys

print("\n--- ТЕСТ ИНФРАКРАСНЫХ ДАТЧИКОВ ---")
print("Один из датчиков сейчас висит на вашем кастомном порту клешни.")
print("Подвигайте рукой ПЕРЕД клешней. Тот пин, который станет 0 - правильный пин!\n")

try:
    while True:
        ir_r = gpio.digital_read(gpio.IR_R)
        ir_l = gpio.digital_read(gpio.IR_L)
        ir_m = gpio.digital_read(gpio.IR_M)
        irf_r = gpio.digital_read(gpio.IRF_R)
        irf_l = gpio.digital_read(gpio.IRF_L)
        
        sys.stdout.write(f"\r[ПИНЫ] IR_R (18): {ir_r} | IR_L (27): {ir_l} | IR_M (22): {ir_m} | IRF_R (25): {irf_r} | IRF_L (1): {irf_l}   ")
        sys.stdout.flush()
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n\nТест завершен.")
