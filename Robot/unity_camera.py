import rospy
import time
from std_msgs.msg import Float32
from xr_servo import Servo
import xr_config as cfg

HAS_SERVO = True
try:
    servo = Servo()
except Exception as e:
    print("Ошибка загрузки модуля сервоприводов для камеры:", e)
    HAS_SERVO = False

SERVO_PAN = 7 # Обычно 7-й (иногда 1-й) сервопривод отвечает за поворот башни/веб-камеры

def callback(msg):
    if not HAS_SERVO: return
    # msg.data приходит отнормированным (-1.0 (лево) до 1.0 (право))
    # Нам нужно конвертировать это в серво-угол 0..180 градусов (где 90 это центр)
    
    yaw = max(-1.0, min(1.0, msg.data)) # Защита (Clamp)
    angle = int(yaw * 90.0 + 90.0)
    
    servo.set(SERVO_PAN, angle)

def listener():
    rospy.init_node('unity_bridge_camera', anonymous=True)
    
    # Инициализация (поворот по центру)
    if HAS_SERVO:
        servo.set(SERVO_PAN, 90)
        
    rospy.Subscriber("/cmd_camera_pan", Float32, callback)
    print("SERVO_PAN (Microservice Camera) запущен! Ждет топик /cmd_camera_pan")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
