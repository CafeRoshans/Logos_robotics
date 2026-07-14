#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion
import time

current_uz_cm = -1.0

def sensor_cb(msg):
    global current_uz_cm
    current_uz_cm = msg.x * 100.0

def main():
    rospy.init_node('test_uz_vs_camera', anonymous=True)
    pub = rospy.Publisher('/cmd_camera_pan', Float32, queue_size=1)
    rospy.Subscriber('/sensor/data', Quaternion, sensor_cb)
    
    print("Подключение к ROS топикам... (unity_master.py должен быть запущен)")
    time.sleep(1.5) # Ждем подключения
    
    print("=" * 50)
    print("ДИАГНОСТИКА: УЗ датчик vs угол камеры (через ROS)")
    print("=" * 50)
    print(f"{'Yaw (Unity)':>12} | {'Angle (Серво)':>15} | {'УЗ дист (cm)':>15} | {'Статус':>10}")
    print("-" * 50)

    problem_angles = []

    # yaw в Unity идет от -1 до 1.
    # В unity_master.py формула: target = 90 - (yaw * 90)
    # Поэтому: yaw = (90 - target) / 90
    for target_angle in range(0, 181, 10):
        yaw = (90 - target_angle) / 90.0
        pub.publish(Float32(data=yaw))
        
        # Ждем пока серво доедет и УЗ обновится
        time.sleep(1.0)
        
        dist = current_uz_cm
        
        status = "✅ OK"
        if dist > 0 and dist < 30:
            status = "🔴 ПЛАТА?!"
            problem_angles.append((target_angle, dist))
        elif dist > 0 and dist < 60:
            status = "⚠️ БЛИЗКО"
            
        print(f"{yaw:>12.2f} | {target_angle:>12}° | {dist:>12.1f}cm | {status:>10}")

    # Возвращаем в центр
    pub.publish(Float32(data=0.0))
    time.sleep(1)

    print("=" * 50)
    if problem_angles:
        print(f"⚠️ ПРОБЛЕМНЫЕ УГЛЫ ({len(problem_angles)}):")
        for ang, dist in problem_angles:
            print(f"   Угол {ang}° → УЗ = {dist:.1f}cm (попадает в конструкцию!)")
        
        safe_min = min(a for a, _ in problem_angles)
        safe_max = max(a for a, _ in problem_angles)
        print(f"\n💡 РЕКОМЕНДАЦИЯ: избегать углы {safe_min}°..{safe_max}°")
    else:
        print("✅ УЗ датчик не ловит конструкцию ни на одном угле камеры.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
