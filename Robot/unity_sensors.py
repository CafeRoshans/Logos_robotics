#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Vector3
import time

# Подключаем папку с драйверами
sys.path.append('/root/XiaoRGeek')

try:
    import xr_gpio as gpio
    from xr_ultrasonic import Ultrasonic
    HAS_SENSORS = True
    us = Ultrasonic()
except ImportError:
    HAS_SENSORS = False
    print("ОШИБКА: Драйверы сенсоров не найдены в /root/XiaoRGeek")

def sensor_publisher():
    rospy.init_node('unity_bridge_sensors', anonymous=True)
    
    # Мы используем Vector3 для передачи всех сенсоров сразу (чтобы не генерировать кастомные типы в Unity):
    # x = Ультразвук (метры)
    # y = Левый ИК (1 = препятствие, 0 = чисто)
    # z = Правый ИК (1 = препятствие, 0 = чисто)
    pub = rospy.Publisher('/sensor/data', Vector3, queue_size=10)
    
    # 10 Гц достаточно для телеметрии без забивания канала
    rate = rospy.Rate(10) 

    print("--- [ВЕРСИЯ 1] МИКРОСЕРВИС СЕНСОРОВ ЗАПУЩЕН ---")

    while not rospy.is_shutdown():
        if HAS_SENSORS:
            msg = Vector3()
            
            # Читаем УЗ
            dist_cm = us.get_distance()
            # Если 0, значит больше 5 метров или сбой. Принимаем как 5.0
            dist_m = dist_cm / 100.0 if dist_cm > 0 else 5.0 
            msg.x = dist_m
            
            # Читаем ИК
            # В xr_infrared 0 = низкий уровень = обнаружение
            ir_l = 1 if gpio.digital_read(gpio.IR_L) == 0 else 0
            ir_r = 1 if gpio.digital_read(gpio.IR_R) == 0 else 0
            
            msg.y = float(ir_l)
            msg.z = float(ir_r)
            
            pub.publish(msg)
            
        rate.sleep()

if __name__ == '__main__':
    try:
        sensor_publisher()
    except rospy.ROSInterruptException:
        pass
