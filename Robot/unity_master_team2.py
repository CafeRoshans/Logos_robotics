#!/usr/bin/env python3
import sys
import rospy
import time
import traceback
import atexit
from geometry_msgs.msg import Twist, Vector3, Quaternion
from std_msgs.msg import Int32, Float32

# Добавляем путь к библиотекам XiaoR Geek
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
        # Ультразвук уже глобально инициализировал серво в собственном драйвере!
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


# ==========================================
# КОНФИГУРАЦИЯ И ЛОГИКА МОТОРОВ
# ==========================================
L = 0.15  
MAX_SPEED_M_S = 0.5 
PWM_CONVERSION_FACTOR = 100.0 / MAX_SPEED_M_S
MIN_MOTOR_PWM = 35

# --- SOFT-START: Защита от пускового тока и Back-EMF ---
# Максимальное изменение PWM за 1 тик. При 50Hz: разгон 0→100% за ~0.14 сек.
MAX_PWM_STEP = 15
prev_pwm_left = 0.0
prev_pwm_right = 0.0
pwm_pub = None
prev_ang_z = 0.0

def clamp_pwm(val):
    return max(min(val, 100.0), -100.0)

def set_motors_pwm(pwm_left, pwm_right):
    global prev_pwm_left, prev_pwm_right, pwm_pub
    if not HAS_GPIO: return
    
    # Левый мотор физически инвертирован (подтверждено тестом с клавиатуры 05.06.2026):
    # Без инверсии W(gas=+1) → робот поворачивает вместо езды вперёд.
    pwm_left = -pwm_left

    # v18 FIX: HARD STOP — если оба таргета нулевые, стопим мгновенно без рампы.
    # Без этого рампа давала 3-4 тика остаточного PWM, а MIN_MOTOR_PWM бустил его до 35.
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
    
    # --- SOFT-START: Ограничиваем скорость нарастания ---
    delta_l = pwm_left - prev_pwm_left
    if abs(delta_l) > MAX_PWM_STEP:
        pwm_left = prev_pwm_left + (MAX_PWM_STEP if delta_l > 0 else -MAX_PWM_STEP)
    
    delta_r = pwm_right - prev_pwm_right
    if abs(delta_r) > MAX_PWM_STEP:
        pwm_right = prev_pwm_right + (MAX_PWM_STEP if delta_r > 0 else -MAX_PWM_STEP)
    
    prev_pwm_left = pwm_left
    prev_pwm_right = pwm_right
    
    abs_l = abs(pwm_left)
    # FIX: Если PWM ниже мёртвой зоны мотора — СТОП, а не буст до 35.
    # Без этого при steering>0.3 один мотор получал PWM=-2 → бустился до -35 → робот крутился.
    MOTOR_DEAD_ZONE = 10  # PWM ниже 10% — мотор всё равно не крутится, ставим 0
    if abs_l < MOTOR_DEAD_ZONE:
        abs_l = 0
    elif abs_l < MIN_MOTOR_PWM:
        abs_l = MIN_MOTOR_PWM
    abs_r = abs(pwm_right)
    if abs_r < MOTOR_DEAD_ZONE:
        abs_r = 0
    elif abs_r < MIN_MOTOR_PWM:
        abs_r = MIN_MOTOR_PWM

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
            # Восстанавливаем логический знак левого мотора (так как он был инвертирован в начале функции)
            logical_left = -pwm_left if abs_l > 0 else 0.0
            logical_right = pwm_right if abs_r > 0 else 0.0
            pwm_pub.publish(Vector3(float(logical_left), float(logical_right), 0.0))
        except:
            pass

SAFETY_STOP_CM = 50  # Локальный стоп если УЗ < 50см и робот едет ВПЕРЁД

def vel_callback(data):
    global filtered_cm, last_cmd_vel_time, prev_ang_z
    last_cmd_vel_time = time.time()
    
    # Дифференциальный привод: v = linear ± angular * TURN_K
    # v27: Снижен TURN_K до 0.25 для плавности поворота, MAX_LINEAR = 0.25 (без изменений)
    TURN_K = 0.25
    MAX_LINEAR = 0.25  # Ограничение: модель даёт gas=1.0, но робот едет max 50%

    lin_x = max(min(data.linear.x, MAX_LINEAR), -MAX_LINEAR)
    
    # EMA сглаживание для угловой скорости (steering)
    EMA_STEER = 0.40
    ang_z = EMA_STEER * data.angular.z + (1.0 - EMA_STEER) * prev_ang_z
    prev_ang_z = ang_z

    v_left  = lin_x + (ang_z * TURN_K)
    v_right = lin_x - (ang_z * TURN_K)
    pwm_left = clamp_pwm(v_left * PWM_CONVERSION_FACTOR)
    pwm_right = clamp_pwm(v_right * PWM_CONVERSION_FACTOR)
    
    set_motors_pwm(pwm_left, pwm_right)

# --- WATCHDOG: Если Unity отключился, стопим моторы ---
last_cmd_vel_time = time.time()
WATCHDOG_TIMEOUT = 0.5  # секунд без команды → СТОП

def watchdog_callback(event):
    global prev_pwm_left, prev_pwm_right
    if time.time() - last_cmd_vel_time > WATCHDOG_TIMEOUT:
        if abs(prev_pwm_left) > 0 or abs(prev_pwm_right) > 0:
            print("⚠️ WATCHDOG: Нет /cmd_vel %.1f сек! АВАРИЙНАЯ ОСТАНОВКА!" % WATCHDOG_TIMEOUT)
            set_motors_pwm(0, 0)
            prev_pwm_left = 0.0
            prev_pwm_right = 0.0


# ==========================================
# КОНФИГУРАЦИЯ И ЛОГИКА СЕРВОМОТОРОВ
# ==========================================
SERVO_BASE = 1      
SERVO_SHOULDER = 2  
SERVO_ELBOW = 3     
SERVO_CLAW = 4      
SERVO_CAMERA_PAN = 7

ANGLE_BASE_CENTER = 90
ANGLE_SHOULDER_UP = 90
ANGLE_ELBOW_UP = 90
ANGLE_SHOULDER_DOWN = 20
ANGLE_ELBOW_DOWN = 130
ANGLE_CLAW_OPEN = 50    
ANGLE_CLAW_CLOSE = 89   

def init_arm():
    if not HAS_SERVO: return
    print("Инициализация начальной позы манипулятора и камеры...")
    servo.set(SERVO_BASE, ANGLE_BASE_CENTER)
    time.sleep(0.3)
    servo.set(SERVO_SHOULDER, ANGLE_SHOULDER_UP)
    time.sleep(0.3)
    servo.set(SERVO_ELBOW, ANGLE_ELBOW_UP)
    time.sleep(0.3)
    servo.set(SERVO_CLAW, ANGLE_CLAW_OPEN) 
    time.sleep(0.3)
    servo.set(SERVO_CAMERA_PAN, 90)
    print("Рука поднята, камера отцентрирована. Робот готов!")

def gripper_callback(data):
    cmd = data.data
    if not HAS_SERVO: return
    if cmd == 1:
        # Prepare to grab: опускаем руку и открываем клешню
        servo.set(SERVO_SHOULDER, ANGLE_SHOULDER_DOWN)
        time.sleep(0.2)
        servo.set(SERVO_ELBOW, ANGLE_ELBOW_DOWN)
        time.sleep(0.2)
        servo.set(SERVO_CLAW, ANGLE_CLAW_OPEN)
    elif cmd == 2:
        # Grab: закрываем клешню и поднимаем руку
        servo.set(SERVO_CLAW, ANGLE_CLAW_CLOSE)
        time.sleep(0.5)
        servo.set(SERVO_ELBOW, ANGLE_ELBOW_UP)
        time.sleep(0.2)
        servo.set(SERVO_SHOULDER, ANGLE_SHOULDER_UP)
    elif cmd == 3:
        # Init: стартовая поза
        init_arm()
    elif cmd == 4:
        # Release only: только открыть клешню, рука остаётся поднятой
        # Используется unity_gripper_ir.py после захвата: рука уже поднята, просто разжимаем
        servo.set(SERVO_CLAW, ANGLE_CLAW_OPEN)
        print(f"[GripperCB] cmd=4: клешня открыта ({ANGLE_CLAW_OPEN}°), рука не двигается")

# --- Плавное слежение камеры ---
current_camera_angle = 90  # Текущий угол серво
MAX_CAMERA_STEP = 15       # Макс градусов за один тик (плавное движение)

def camera_callback(data):
    global current_camera_angle
    if not HAS_SERVO: return
    yaw = data.data
    # ИНВЕРСИЯ: Если в Unity камера в одну сторону, а в реальности в другую — меняем знак здесь
    target = 90 - (yaw * 90)
    target = max(0, min(180, target))
    
    # Плавное движение: не больше MAX_CAMERA_STEP градусов за шаг
    diff = target - current_camera_angle
    if abs(diff) > MAX_CAMERA_STEP:
        diff = MAX_CAMERA_STEP if diff > 0 else -MAX_CAMERA_STEP
    
    current_camera_angle += diff
    current_camera_angle = max(0, min(180, current_camera_angle))
    servo.set(SERVO_CAMERA_PAN, int(current_camera_angle))

# ==========================================
# ЧТЕНИЕ СЕНСОРОВ В ТАЙМЕРЕ
# ==========================================
# --- Фильтрация датчиков ---
us_history = [100.0, 100.0, 100.0]
filtered_cm = 500.0 
ir_l_history = [0, 0, 0, 0, 0]
ir_r_history = [0, 0, 0, 0, 0]
ir_gripper_history = [0, 0, 0, 0, 0]  # ИК датчик клешни (gpio.IR_M, pin 22)

def sensor_timer_callback(event):
    global us_history, ir_l_history, ir_r_history, ir_gripper_history, filtered_cm
    if not HAS_SENSORS or sensor_pub is None: return
    try:
        msg = Quaternion()
        # 1. Ультразвук с медианным фильтром (окно 3)
        dist_cm = us.get_distance()
        if dist_cm <= 0 or dist_cm > 500: dist_cm = 500.0
        
        us_history.pop(0)
        us_history.append(dist_cm)
        
        # Медиана убирает одиночные "вылеты" (0 или 500)
        sorted_us = sorted(us_history)
        filtered_cm = sorted_us[1]
        msg.x = filtered_cm / 100.0
        
        # 2. ИК сенсоры (ИНВЕРСИЯ: Свапнуты L/R для верного отображения)
        ir_l = 1 if gpio.digital_read(gpio.IRF_R) == 0 else 0
        ir_r = 1 if gpio.digital_read(gpio.IRF_L) == 0 else 0
        
        ir_l_history.pop(0)
        ir_l_history.append(ir_l)
        ir_r_history.pop(0)
        ir_r_history.append(ir_r)
        
        # Только если последние 2 из 3 значений == 1, считаем препятствие (фильтр дребезга)
        msg.y = float(1 if sum(ir_l_history[-3:]) >= 2 else 0)
        msg.z = float(1 if sum(ir_r_history[-3:]) >= 2 else 0)
        
        # 3. ИК датчик клешни (gpio.IR_M, pin 22)
        # ИСПРАВЛЕНО: ранее msg.w = 0.0 (хардкод), из-за чего автозахват на реальном
        # роботе никогда не срабатывал. Теперь читаем реальный пин.
        ir_gripper = 1 if gpio.digital_read(gpio.IR_M) == 0 else 0
        ir_gripper_history.pop(0)
        ir_gripper_history.append(ir_gripper)
        msg.w = float(1 if sum(ir_gripper_history[-3:]) >= 2 else 0)
        
        sensor_pub.publish(msg)
        
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
    
    rospy.Subscriber('/cmd_vel', Twist, vel_callback)
    
    if HAS_SERVO:
        rospy.Subscriber('/cmd_gripper', Int32, gripper_callback)
        rospy.Subscriber('/cmd_camera_pan', Float32, camera_callback)
        init_arm() 
        
    if HAS_SENSORS:
        sensor_pub = rospy.Publisher('/sensor/data', Quaternion, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), sensor_timer_callback) 
        
    pwm_pub = rospy.Publisher('/sensor/pwm', Vector3, queue_size=10)

    # Watchdog: каждые 0.2 сек проверяем, жив ли /cmd_vel
    rospy.Timer(rospy.Duration(0.2), watchdog_callback)

    rospy.spin()

# --- АВАРИЙНАЯ ОСТАНОВКА МОТОРОВ ПРИ ВЫХОДЕ ---
def emergency_stop():
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
