#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Int32
import time

# Подключаем папку с драйверами
sys.path.append('/root/XiaoRGeek')

import traceback

try:
    from xr_servo import Servo
    HAS_SERVO = True
    servo = Servo()
except Exception as e:
    HAS_SERVO = False
    print("=== ОШИБКА ДРАЙВЕРА СЕРВОМОТОРОВ ===")
    print(traceback.format_exc())
    print("====================================")

# --- КАЛИБРОВКА СЕРВОПРИВОДОВ ---
# Настройте эти значения под вашу физическую клешню XiaoRGeek!
SERVO_BASE = 1      # Поворот вправо/влево
SERVO_SHOULDER = 2  # Плечо (вверх/вниз)
SERVO_ELBOW = 3     # Локоть (вверх/вниз)
SERVO_CLAW = 4      # Сама клешня (захват)
ANGLE_BASE_CENTER = 90     # Центральное положение базы

# Значения для верхнего (походного) положения
ANGLE_SHOULDER_UP = 90     # Плечо поднято
ANGLE_ELBOW_UP = 90        # Локоть поднят

# Значения для нижнего (рабочего) положения
ANGLE_SHOULDER_DOWN = 20   # Значение опущенного плеча
ANGLE_ELBOW_DOWN = 130     # Значение опущенного локтя

# Значения для самой клешни
ANGLE_CLAW_OPEN = 10       # Угол открытой клешни
ANGLE_CLAW_CLOSE = 90      # Угол закрытой клешни (захват мяча)

def init_arm():
    if not HAS_SERVO: return
    print("Инициализация манипулятора (поднятие в стартовое положение)...")
    servo.set(SERVO_BASE, ANGLE_BASE_CENTER)
    time.sleep(0.3)
    servo.set(SERVO_SHOULDER, ANGLE_SHOULDER_UP)
    time.sleep(0.3)
    servo.set(SERVO_ELBOW, ANGLE_ELBOW_UP)
    time.sleep(0.3)
    servo.set(SERVO_CLAW, ANGLE_CLAW_CLOSE) # При езде лучше держать закрытой
    time.sleep(0.3)
    print("Рука поднята. Робот готов к маневрам.")

def callback(msg):
    if not HAS_SERVO: return
    
    cmd = msg.data
    # 0 = ничего
    # 1 = приготовиться к захвату (опустить руку и открыть клешню)
    if cmd == 1:
        servo.set(SERVO_SHOULDER, ANGLE_SHOULDER_DOWN)
        time.sleep(0.2)
        servo.set(SERVO_ELBOW, ANGLE_ELBOW_DOWN)
        time.sleep(0.2)
        servo.set(SERVO_CLAW, ANGLE_CLAW_OPEN)
        print("Команда Акшена: Рука ОПУЩЕНА, клешня ОТКРЫТА")
        
    # 2 = схватить и поднять мяч
    elif cmd == 2:
        servo.set(SERVO_CLAW, ANGLE_CLAW_CLOSE)
        time.sleep(0.5) # Ждем полсекунды, чтобы клешня плотно сжала мяч
        servo.set(SERVO_ELBOW, ANGLE_ELBOW_UP)
        time.sleep(0.2)
        servo.set(SERVO_SHOULDER, ANGLE_SHOULDER_UP)
        print("Команда Акшена: Мяч ЗАХВАЧЕН, рука ПОДНЯТА")

    # 3 = стартовая поза
    elif cmd == 3:
        print("Unity: Возврат в стартовую позу...")
        init_arm()

def listener():
    rospy.init_node('unity_bridge_gripper', anonymous=True)
    init_arm()
    rospy.Subscriber("/cmd_gripper", Int32, callback)
    print("--- [ВЕРСИЯ 1] МИКРОСЕРВИС КЛЕШНИ ЗАПУЩЕН ---")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
