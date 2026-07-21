#!/usr/bin/env python3
import sys
import os
import rospy
import time
import traceback
import atexit
from datetime import datetime
from geometry_msgs.msg import Twist, Vector3, Quaternion
from std_msgs.msg import Int32, Float32

sys.path.append('/root/XiaoRGeek')

print("--- Инициализация XiaoR драйверов ---")

HAS_GPIO = False
try:
    import xr_gpio as gpio
    HAS_GPIO = True
    print("✅ Драйвер моторов (xr_gpio) загружен успешно!")
except Exception as e:
    print("❌ ОШИБКА загрузки xr_gpio:")
    print(traceback.format_exc())

HAS_SENSORS = False
us = None
try:
    from xr_ultrasonic import Ultrasonic
    us = Ultrasonic()
    HAS_SENSORS = True
    print("✅ Драйвер Ultrasonic загружен успешно!")
except Exception as e:
    print("❌ ОШИБКА загрузки xr_ultrasonic:")
    print(traceback.format_exc())

HAS_SERVO = False
try:
    if HAS_SENSORS:
        import xr_ultrasonic
        servo = xr_ultrasonic.servo
        HAS_SERVO = True
        print("✅ Драйвер серво переиспользован из Ultrasonic (защита от двойной I2C)!")
    else:
        from xr_servo import Servo
        servo = Servo()
        HAS_SERVO = True
        print("✅ Драйвер сервомоторов (xr_servo) загружен успешно!")
except Exception as e:
    print("❌ ОШИБКА загрузки xr_servo:")
    print(traceback.format_exc())

import config as cfg

# ==========================================
# CSV-ЛОГИРОВАНИЕ
# ==========================================
LOG_DIR = "/tmp/unity_logs"
os.makedirs(LOG_DIR, exist_ok=True)
_log_filename = os.path.join(LOG_DIR, f"master_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
_csv_file = None
_log_t0 = time.time()

_CSV_HEADER = (
    "t,event,"
    "cmd_lin_x,cmd_ang_z,ang_z_ema,"
    "v_left,v_right,pwm_l,pwm_r,pwm_l_after_ramp,pwm_r_after_ramp,"
    "uz_raw,uz_filt,ir_l_raw,ir_r_raw,ir_g_raw,ir_l,ir_r,ir_g,"
    "cam_angle,gripper_cmd"
)

def _init_log():
    global _csv_file
    _csv_file = open(_log_filename, "w")
    _csv_file.write(_CSV_HEADER + "\n")
    print(f"📝 CSV log: {_log_filename}")

def _log(event, **kw):
    if _csv_file is None:
        return
    t = time.time() - _log_t0
    row = [
        f"{t:.3f}", event,
        kw.get("lin_x", ""), kw.get("ang_z_raw", ""), kw.get("ang_z_ema", ""),
        kw.get("v_left", ""), kw.get("v_right", ""),
        kw.get("pwm_l", ""), kw.get("pwm_r", ""),
        kw.get("pwm_l_ramp", ""), kw.get("pwm_r_ramp", ""),
        kw.get("uz_raw", ""), kw.get("uz_filt", ""),
        kw.get("ir_l_raw", ""), kw.get("ir_r_raw", ""), kw.get("ir_g_raw", ""),
        kw.get("ir_l", ""), kw.get("ir_r", ""), kw.get("ir_g", ""),
        kw.get("cam_angle", ""), kw.get("gripper_cmd", ""),
    ]
    _csv_file.write(",".join(str(v) for v in row) + "\n")
    _csv_file.flush()


# ==========================================
# ЛОГИКА МОТОРОВ
# ==========================================
prev_pwm_left = 0.0
prev_pwm_right = 0.0
pwm_pub = None
prev_ang_z = 0.0

def clamp_pwm(val):
    return max(min(val, 100.0), -100.0)

def set_motors_pwm(pwm_left, pwm_right):
    global prev_pwm_left, prev_pwm_right, pwm_pub
    if not HAS_GPIO: return

    if cfg.MOTOR_INVERT_LEFT:
        pwm_left = -pwm_left
    if cfg.MOTOR_INVERT_RIGHT:
        pwm_right = -pwm_right

    if abs(pwm_left) < 0.5 and abs(pwm_right) < 0.5:
        prev_pwm_left = 0.0
        prev_pwm_right = 0.0
        gpio.digital_write(gpio.IN1, 0)
        gpio.digital_write(gpio.IN2, 0)
        gpio.digital_write(gpio.IN3, 0)
        gpio.digital_write(gpio.IN4, 0)
        gpio.ena_pwm(0)
        gpio.enb_pwm(0)
        if pwm_pub is not None:
            try:
                pwm_pub.publish(Vector3(0.0, 0.0, 0.0))
            except:
                pass
        return

    delta_l = pwm_left - prev_pwm_left
    if abs(delta_l) > cfg.MAX_PWM_STEP:
        pwm_left = prev_pwm_left + (cfg.MAX_PWM_STEP if delta_l > 0 else -cfg.MAX_PWM_STEP)

    delta_r = pwm_right - prev_pwm_right
    if abs(delta_r) > cfg.MAX_PWM_STEP:
        pwm_right = prev_pwm_right + (cfg.MAX_PWM_STEP if delta_r > 0 else -cfg.MAX_PWM_STEP)

    prev_pwm_left = pwm_left
    prev_pwm_right = pwm_right

    abs_l = abs(pwm_left)
    if abs_l < cfg.MOTOR_DEAD_ZONE:
        abs_l = 0
    elif abs_l < cfg.MIN_MOTOR_PWM:
        abs_l = cfg.MIN_MOTOR_PWM

    abs_r = abs(pwm_right)
    if abs_r < cfg.MOTOR_DEAD_ZONE:
        abs_r = 0
    elif abs_r < cfg.MIN_MOTOR_PWM:
        abs_r = cfg.MIN_MOTOR_PWM
    abs_r = min(int(abs_r * cfg.RIGHT_MOTOR_BOOST), 100)

    if int(abs_l) == 0 and int(abs_r) == 0:
        gpio.digital_write(gpio.IN1, 0)
        gpio.digital_write(gpio.IN2, 0)
        gpio.digital_write(gpio.IN3, 0)
        gpio.digital_write(gpio.IN4, 0)
        gpio.ena_pwm(0)
        gpio.enb_pwm(0)
        if pwm_pub is not None:
            try:
                pwm_pub.publish(Vector3(0.0, 0.0, 0.0))
            except:
                pass
        return

    gpio.ena_pwm(int(abs_l))
    gpio.enb_pwm(int(abs_r))

    if pwm_left > 0:
        gpio.digital_write(gpio.IN1, 1)
        gpio.digital_write(gpio.IN2, 0)
    elif pwm_left < 0:
        gpio.digital_write(gpio.IN1, 0)
        gpio.digital_write(gpio.IN2, 1)
    else:
        gpio.digital_write(gpio.IN1, 0)
        gpio.digital_write(gpio.IN2, 0)

    if pwm_right > 0:
        gpio.digital_write(gpio.IN3, 1)
        gpio.digital_write(gpio.IN4, 0)
    elif pwm_right < 0:
        gpio.digital_write(gpio.IN3, 0)
        gpio.digital_write(gpio.IN4, 1)
    else:
        gpio.digital_write(gpio.IN3, 0)
        gpio.digital_write(gpio.IN4, 0)

    if pwm_pub is not None:
        try:
            logical_left = -pwm_left
            logical_right = -pwm_right
            pwm_pub.publish(Vector3(float(logical_left), float(logical_right), 0.0))
        except:
            pass


def vel_callback(data):
    global filtered_cm, last_cmd_vel_time, prev_ang_z
    last_cmd_vel_time = time.time()

    lin_x = max(min(data.linear.x, cfg.MAX_LINEAR), -cfg.MAX_LINEAR)

    ang_z = cfg.EMA_STEER * data.angular.z + (1.0 - cfg.EMA_STEER) * prev_ang_z
    prev_ang_z = ang_z

    v_left  = lin_x + (ang_z * cfg.TURN_K)
    v_right = lin_x - (ang_z * cfg.TURN_K)
    pwm_left = clamp_pwm(v_left * cfg.PWM_CONVERSION_FACTOR)
    pwm_right = clamp_pwm(v_right * cfg.PWM_CONVERSION_FACTOR)

    set_motors_pwm(pwm_left, pwm_right)

    _log("cmd_vel",
         lin_x=f"{lin_x:.3f}", ang_z_raw=f"{data.angular.z:.3f}", ang_z_ema=f"{ang_z:.3f}",
         v_left=f"{v_left:.3f}", v_right=f"{v_right:.3f}",
         pwm_l=f"{pwm_left:.1f}", pwm_r=f"{pwm_right:.1f}",
         pwm_l_ramp=f"{prev_pwm_left:.1f}", pwm_r_ramp=f"{prev_pwm_right:.1f}")

last_cmd_vel_time = time.time()

def watchdog_callback(event):
    global prev_pwm_left, prev_pwm_right
    if time.time() - last_cmd_vel_time > cfg.WATCHDOG_TIMEOUT:
        if abs(prev_pwm_left) > 0 or abs(prev_pwm_right) > 0:
            print("⚠️ WATCHDOG: Нет /cmd_vel %.1f сек! АВАРИЙНАЯ ОСТАНОВКА!" % cfg.WATCHDOG_TIMEOUT)
            _log("watchdog", pwm_l_ramp=f"{prev_pwm_left:.1f}", pwm_r_ramp=f"{prev_pwm_right:.1f}")
            set_motors_pwm(0, 0)
            prev_pwm_left = 0.0
            prev_pwm_right = 0.0


# ==========================================
# ЛОГИКА СЕРВОМОТОРОВ
# ==========================================
def init_arm():
    if not HAS_SERVO: return
    print("Инициализация начальной позы манипулятора и камеры...")
    servo.set(cfg.SERVO_BASE, cfg.ANGLE_BASE_CENTER)
    time.sleep(0.3)
    servo.set(cfg.SERVO_SHOULDER, cfg.ANGLE_SHOULDER_UP)
    time.sleep(0.3)
    servo.set(cfg.SERVO_ELBOW, cfg.ANGLE_ELBOW_UP)
    time.sleep(0.3)
    servo.set(cfg.SERVO_CLAW, cfg.ANGLE_CLAW_OPEN)
    time.sleep(0.3)
    servo.set(cfg.SERVO_CAMERA_LOW, 90)
    print("Рука поднята, камера отцентрирована. Робот готов!")

def gripper_callback(data):
    cmd = data.data
    _log("gripper", gripper_cmd=cmd)
    if not HAS_SERVO: return
    if cmd == 1:
        servo.set(cfg.SERVO_BASE, cfg.ANGLE_BASE_CENTER)
        time.sleep(0.1)
        servo.set(cfg.SERVO_SHOULDER, cfg.ANGLE_SHOULDER_DOWN)
        time.sleep(0.1)
        servo.set(cfg.SERVO_ELBOW, cfg.ANGLE_ELBOW_DOWN)
        time.sleep(0.1)
        servo.set(cfg.SERVO_CLAW, cfg.ANGLE_CLAW_OPEN)
    elif cmd == 2:
        servo.set(cfg.SERVO_BASE, cfg.ANGLE_BASE_CENTER)
        time.sleep(0.1)
        servo.set(cfg.SERVO_CLAW, cfg.ANGLE_CLAW_CLOSE)
        time.sleep(0.1)
        servo.set(cfg.SERVO_ELBOW, cfg.ANGLE_ELBOW_UP)
        time.sleep(0.1)
        servo.set(cfg.SERVO_SHOULDER, cfg.ANGLE_SHOULDER_UP)
    elif cmd == 3:
        init_arm()
    elif cmd == 4:
        servo.set(cfg.SERVO_CLAW, cfg.ANGLE_CLAW_OPEN)
        print(f"[GripperCB] cmd=4: клешня открыта ({cfg.ANGLE_CLAW_OPEN}°), рука не двигается")

current_camera_angle = 90

def camera_callback(data):
    global current_camera_angle
    if not HAS_SERVO: return
    yaw = data.data
    target = 90 - (yaw * 90)
    target = max(0, min(180, target))

    diff = target - current_camera_angle
    if abs(diff) > cfg.MAX_CAMERA_STEP:
        diff = cfg.MAX_CAMERA_STEP if diff > 0 else -cfg.MAX_CAMERA_STEP

    current_camera_angle += diff
    current_camera_angle = max(0, min(180, current_camera_angle))
    servo.set(cfg.SERVO_CAMERA_PAN, int(current_camera_angle))
    _log("camera", cam_angle=f"{current_camera_angle:.0f}")


# ==========================================
# ЧТЕНИЕ СЕНСОРОВ
# ==========================================
us_history = [100.0, 100.0, 100.0]
filtered_cm = 500.0
ir_l_history = [0, 0, 0, 0, 0]
ir_r_history = [0, 0, 0, 0, 0]
ir_gripper_history = [0, 0, 0, 0, 0]

def sensor_timer_callback(event):
    global us_history, ir_l_history, ir_r_history, ir_gripper_history, filtered_cm
    if not HAS_SENSORS or sensor_pub is None: return
    try:
        msg = Quaternion()
        dist_cm = us.get_distance()
        if dist_cm <= 0 or dist_cm > 500.0: dist_cm = 500.0

        us_history.pop(0)
        us_history.append(dist_cm)

        sorted_us = sorted(us_history)
        filtered_cm = sorted_us[1]
        msg.x = filtered_cm / 100.0

        ir_l = 1 if gpio.digital_read(cfg.PIN_IR_LEFT) == 0 else 0
        ir_r = 1 if gpio.digital_read(cfg.PIN_IR_RIGHT) == 0 else 0

        ir_l_history.pop(0)
        ir_l_history.append(ir_l)
        ir_r_history.pop(0)
        ir_r_history.append(ir_r)

        msg.y = float(1 if sum(ir_l_history[-3:]) >= 2 else 0)
        msg.z = float(1 if sum(ir_r_history[-3:]) >= 2 else 0)

        ir_gripper = 1 if gpio.digital_read(cfg.PIN_IR_GRIPPER) == 0 else 0
        ir_gripper_history.pop(0)
        ir_gripper_history.append(ir_gripper)
        msg.w = float(1 if sum(ir_gripper_history[-3:]) >= 2 else 0)

        sensor_pub.publish(msg)

        _log("sensor",
             uz_raw=f"{dist_cm:.1f}", uz_filt=f"{filtered_cm:.1f}",
             ir_l_raw=ir_l, ir_r_raw=ir_r, ir_g_raw=ir_gripper,
             ir_l=int(msg.y), ir_r=int(msg.z), ir_g=int(msg.w))

        if msg.w == 1.0:
            print(f"🎯 ИК КЛЕШНЯ: мяч обнаружен! (UZ={filtered_cm:.1f}cm, IR_L={int(msg.y)}, IR_R={int(msg.z)})")
    except Exception as e:
        print(f"❌ ОШИБКА В ТАЙМЕРЕ СЕНСОРОВ: {e}")


# ==========================================
# ГЛАВНЫЙ БЛОК ROS
# ==========================================
def listener():
    global sensor_pub, pwm_pub
    rospy.init_node('unity_robot_master', anonymous=True)
    _init_log()

    rospy.Subscriber('/cmd_vel', Twist, vel_callback)

    if HAS_SERVO:
        rospy.Subscriber('/cmd_gripper', Int32, gripper_callback)
        rospy.Subscriber('/cmd_camera_pan', Float32, camera_callback)
        init_arm()

    if HAS_SENSORS:
        sensor_pub = rospy.Publisher('/sensor/data', Quaternion, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), sensor_timer_callback)

    pwm_pub = rospy.Publisher('/sensor/pwm', Vector3, queue_size=10)

    rospy.Timer(rospy.Duration(0.2), watchdog_callback)

    rospy.spin()

def emergency_stop():
    if _csv_file:
        try:
            _csv_file.close()
            print(f"📝 Log saved: {_log_filename}")
        except:
            pass
    try:
        if HAS_GPIO:
            gpio.digital_write(gpio.IN1, 0)
            gpio.digital_write(gpio.IN2, 0)
            gpio.digital_write(gpio.IN3, 0)
            gpio.digital_write(gpio.IN4, 0)
            gpio.ena_pwm(0)
            gpio.enb_pwm(0)
    except:
        pass

atexit.register(emergency_stop)

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        emergency_stop()
