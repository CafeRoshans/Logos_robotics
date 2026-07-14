#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Twist

# Подключаем папку с драйверами
sys.path.append('/root/XiaoRGeek')

try:
    import xr_gpio as gpio
    HAS_DRIVER = True
except ImportError:
    HAS_DRIVER = False
    print("ОШИБКА: Драйвер xr_gpio не найден в /root/XiaoRGeek")

# --- Настройки дифференциального привода ---
# Ширина колесной базы (L) в метрах. Если робот будет поворачивать слишком резко, увеличьте это значение.
L = 0.20  
# Максимальная физическая скорость робота (м/с). 
# Нужна для перевода из единиц Twist (м/с) в ШИМ от 0 до 100%
MAX_SPEED_M_S = 0.5 
PWM_CONVERSION_FACTOR = 100.0 / MAX_SPEED_M_S

# Мертвая зона моторов (в процентах ШИМ)
MIN_MOTOR_PWM = 20

def clamp_pwm(val):
    """Обрезаем ШИМ в диапазон от -100 до 100"""
    return max(min(val, 100.0), -100.0)

def set_motors_pwm(pwm_left, pwm_right):
    """Принимает ШИМ для левой и правой гусеницы (от -100 до 100)"""
    if not HAS_DRIVER:
        print(f"DEBUG NO GPIO: Left PWM={pwm_left:.1f}, Right PWM={pwm_right:.1f}")
        return

    # Применяем мертвую зону (моторы не стартуют на слишком малом ШИМ)
    abs_l = abs(pwm_left)
    if 0 < abs_l < MIN_MOTOR_PWM: 
        abs_l = MIN_MOTOR_PWM
    
    abs_r = abs(pwm_right)
    if 0 < abs_r < MIN_MOTOR_PWM: 
        abs_r = MIN_MOTOR_PWM

    # Физический тормоз: если ШИМ = 0, принудительно глушим пины направления!
    # Иначе драйвер может пропускать ток (фантомный левый спин)
    if int(abs_l) == 0 and int(abs_r) == 0:
        gpio.digital_write(gpio.IN1, 0)
        gpio.digital_write(gpio.IN2, 0)
        gpio.digital_write(gpio.IN3, 0)
        gpio.digital_write(gpio.IN4, 0)
        gpio.ena_pwm(0)
        gpio.enb_pwm(0)
        return

    # Устанавливаем мощность через ENA/ENB
    gpio.ena_pwm(int(abs_l))
    gpio.enb_pwm(int(abs_r))

    # Устанавливаем направление для левой гусеницы (IN1, IN2)
    # IN1=1, IN2=0 -> Вперед
    if pwm_left > 0:
        gpio.digital_write(gpio.IN1, 1)
        gpio.digital_write(gpio.IN2, 0)
    elif pwm_left < 0:
        gpio.digital_write(gpio.IN1, 0)
        gpio.digital_write(gpio.IN2, 1)
    else:
        gpio.digital_write(gpio.IN1, 0)
        gpio.digital_write(gpio.IN2, 0)

    # Устанавливаем направление для правой гусеницы (IN3, IN4)
    if pwm_right > 0:
        gpio.digital_write(gpio.IN3, 0)
        gpio.digital_write(gpio.IN4, 1)
    elif pwm_right < 0:
        gpio.digital_write(gpio.IN3, 1)
        gpio.digital_write(gpio.IN4, 0)
    else:
        gpio.digital_write(gpio.IN3, 0)
        gpio.digital_write(gpio.IN4, 0)

def callback(msg):
    linear_v = msg.linear.x
    angular_z = msg.angular.z

    # Вычисление кинематики идеального дифференциального привода с коэффициентом L.
    # ВНИМАНИЕ: В классическом ROS (z - это поворот влево), формула: V_left = v - z*L/2
    # Однако в Unity 3D положительный steer по X генерирует поворот ВПРАВО! 
    # Поэтому мы слегка адаптируем знаки, чтобы робот поворачивал в правильную сторону относительно мира Unity.
    # Если angular_z > 0 (команда Unity на поворот вправо), левая гусеница должна крутиться быстрее:
    v_left  = linear_v + (angular_z * L / 2.0)
    v_right = linear_v - (angular_z * L / 2.0)

    # Перевод скоростей (м/с) в ШИМ-сигнал (-100...100)
    pwm_left = clamp_pwm(v_left * PWM_CONVERSION_FACTOR)
    pwm_right = clamp_pwm(v_right * PWM_CONVERSION_FACTOR)

    set_motors_pwm(pwm_left, pwm_right)

def listener():
    rospy.init_node('unity_bridge_motors', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    print("--- [ВЕРСИЯ 3] МОСТ UNITY-РОБОТ ЗАПУЩЕН ---")
    print("--- КИНЕМАТИКА ДИФФЕРЕНЦИАЛЬНОГО ПРИВОДА (ШИМ-Clamp) ---")
    print("Ожидание данных по Wi-Fi...")
    rospy.spin()

if __name__ == '__main__':
    listener()
