#!/usr/bin/env python3
"""
unity_gripper_ir.py — ROS-узел автоматической клешни по ИК датчику
===================================================================
Работает ПАРАЛЛЕЛЬНО с unity_master.py при запуске через start_robot.sh.

Принцип работы:
  - Читает IR_M (GPIO pin 22, IRM на плате расширения) с частотой POLL_HZ
  - Публикует состояние в /sensor/gripper_ir (Int32) → Unity видит gripperIR
  - Публикует /cmd_gripper (Int32) → unity_master.py физически двигает серво:
      cmd=2 → закрыть клешню + поднять руку
      cmd=1 → открыть клешню

ВАЖНО: НЕ управляет серво напрямую (это делает unity_master.py).
       НЕ импортирует xr_servo/xr_ultrasonic (они уже заняты unity_master.py,
       повторный bind() на порт 2005 даст OSError: Address already in use).

Запуск вручную (после ./start_robot.sh):
  docker exec -it xiao_ros_brain bash
  source /opt/ros/noetic/setup.bash
  python3 /root/unity_gripper_ir.py

Проверка логов:
  docker exec xiao_ros_brain cat /tmp/gripper_ir.log
"""

import sys
import time
import traceback

sys.path.append('/root/XiaoRGeek')

# --- ROS ---
import rospy
from std_msgs.msg import Int32

# --- GPIO (только для чтения пина, НЕ инициализируем серво) ---
HAS_GPIO = False
try:
    import xr_gpio as gpio
    HAS_GPIO = True
    print("✅ [GripperIR] xr_gpio загружен — пин IR_M (22) доступен")
except Exception as e:
    print(f"❌ [GripperIR] Ошибка загрузки xr_gpio: {e}")
    print(traceback.format_exc())

# =============================================
# НАСТРОЙКИ — должны совпадать с unity_master.py
# =============================================
POLL_HZ        = 20    # Частота опроса ИК датчика (раз/сек)
DEBOUNCE_COUNT = 3     # Сколько одинаковых показаний подряд = стабильный сигнал

CMD_GRIPPER_CLOSE = 2  # unity_master.py: закрыть клешню + поднять руку
CMD_GRIPPER_OPEN  = 4  # unity_master.py: только открыть клешню (рука остаётся поднятой)

# =============================================

def read_ir_m():
    """
    Читаем IR_M (pin 22) с pull-up логикой:
      GPIO == 0  →  объект есть   → возвращаем 1
      GPIO == 1  →  свободно      → возвращаем 0
    """
    if not HAS_GPIO:
        return 0
    return 1 if gpio.digital_read(gpio.IR_M) == 0 else 0

def main():
    rospy.init_node('unity_gripper_ir', anonymous=False)

    # /sensor/gripper_ir → VirtualSensors.cs (GripperIRCallback) → gripperIR
    ir_pub = rospy.Publisher('/sensor/gripper_ir', Int32, queue_size=5)

    # /cmd_gripper → unity_master.py (gripper_callback) → физическое серво
    cmd_pub = rospy.Publisher('/cmd_gripper', Int32, queue_size=5)

    rospy.loginfo("=== [GripperIR] Узел запущен ===")
    rospy.loginfo("  Датчик    : IR_M (GPIO pin 22, IRM на плате)")
    rospy.loginfo("  → /sensor/gripper_ir (Int32)  → Unity VirtualSensors")
    rospy.loginfo("  → /cmd_gripper (Int32)         → unity_master.py → серво")
    rospy.loginfo(f"  Частота   : {POLL_HZ} Hz | Debounce: {DEBOUNCE_COUNT} показаний")
    rospy.loginfo("Перекрой ИК датчик → /cmd_gripper=2 (закрыть)")
    rospy.loginfo("Убери руку         → /cmd_gripper=1 (открыть)")

    rate = rospy.Rate(POLL_HZ)
    debounce_buf = [0] * DEBOUNCE_COUNT
    prev_stable  = -1     # Предыдущее стабильное значение (-1 = неизвестно)

    while not rospy.is_shutdown():
        # 1. Читаем пин
        raw = read_ir_m()

        # 2. Скользящий фильтр дребезга
        debounce_buf.pop(0)
        debounce_buf.append(raw)
        stable_ir = 1 if sum(debounce_buf) >= DEBOUNCE_COUNT else 0

        # 3. Публикуем состояние датчика → VirtualSensors.cs в Unity
        ir_pub.publish(Int32(stable_ir))

        # 4. На ИЗМЕНЕНИЕ состояния — шлём команду серво через unity_master.py
        if stable_ir != prev_stable:
            if stable_ir == 1:
                rospy.loginfo("🎯 [GripperIR] ИК СРАБОТАЛ → /cmd_gripper=2 (закрыть клешню)")
                cmd_pub.publish(Int32(CMD_GRIPPER_CLOSE))
            elif prev_stable != -1:  # Не посылаем "открыть" при первом старте
                rospy.loginfo("↕  [GripperIR] ИК освобождён → /cmd_gripper=1 (открыть клешню)")
                cmd_pub.publish(Int32(CMD_GRIPPER_OPEN))
            prev_stable = stable_ir

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
