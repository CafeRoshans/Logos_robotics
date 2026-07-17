#!/usr/bin/env python3
import sys
import socket
import json
import time
import atexit

sys.path.append('/root/XiaoRGeek')

# --- Инициализация Моторов (GPIO) ---
try:
    import xr_gpio as gpio
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False
    print("ОШИБКА: Драйвер xr_gpio не найден!")

L = 0.20  
MAX_SPEED_M_S = 0.5 
PWM_CONVERSION_FACTOR = 100.0 / MAX_SPEED_M_S
MIN_MOTOR_PWM = 20

# --- SOFT-START: Защита от пускового тока и Back-EMF ---
MAX_PWM_STEP = 15
prev_pwm_left = 0.0
prev_pwm_right = 0.0

def clamp_pwm(val):
    return max(min(val, 100.0), -100.0)

def set_motors_pwm(pwm_left, pwm_right):
    global prev_pwm_left, prev_pwm_right
    if not HAS_GPIO: return

    # --- SOFT-START ---
    delta_l = pwm_left - prev_pwm_left
    if abs(delta_l) > MAX_PWM_STEP:
        pwm_left = prev_pwm_left + (MAX_PWM_STEP if delta_l > 0 else -MAX_PWM_STEP)
    delta_r = pwm_right - prev_pwm_right
    if abs(delta_r) > MAX_PWM_STEP:
        pwm_right = prev_pwm_right + (MAX_PWM_STEP if delta_r > 0 else -MAX_PWM_STEP)
    prev_pwm_left = pwm_left
    prev_pwm_right = pwm_right

    abs_l = abs(pwm_left)
    if 0 < abs_l < MIN_MOTOR_PWM: abs_l = MIN_MOTOR_PWM
    abs_r = abs(pwm_right)
    if 0 < abs_r < MIN_MOTOR_PWM: abs_r = MIN_MOTOR_PWM

    if int(abs_l) == 0 and int(abs_r) == 0:
        gpio.digital_write(gpio.IN1, 0)
        gpio.digital_write(gpio.IN2, 0)
        gpio.digital_write(gpio.IN3, 0)
        gpio.digital_write(gpio.IN4, 0)
        gpio.ena_pwm(0)
        gpio.enb_pwm(0)
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
        gpio.digital_write(gpio.IN3, 0)
        gpio.digital_write(gpio.IN4, 1)
    elif pwm_right < 0:
        gpio.digital_write(gpio.IN3, 1)
        gpio.digital_write(gpio.IN4, 0)
    else:
        gpio.digital_write(gpio.IN3, 0)
        gpio.digital_write(gpio.IN4, 0)

def process_twist(linear_v, angular_z):
    v_left  = linear_v + (angular_z * L / 2.0)
    v_right = linear_v - (angular_z * L / 2.0)
    pwm_left = clamp_pwm(v_left * PWM_CONVERSION_FACTOR)
    pwm_right = clamp_pwm(v_right * PWM_CONVERSION_FACTOR)
    set_motors_pwm(pwm_left, pwm_right)

# --- Инициализация Сервомоторов (Клешня и Камера) ---
try:
    from xr_servo import Servo
    HAS_SERVO = True
    servo = Servo()
except ImportError as e:
    HAS_SERVO = False
    print("ОШИБКА: Драйвер xr_servo не найден:", e)

SERVO_BASE = 1      
SERVO_SHOULDER = 2  
SERVO_ELBOW = 3     
SERVO_CLAW = 4      
SERVO_CAMERA_PAN = 7 # Обычно 7 или 8 порт для камеры

ANGLE_BASE_CENTER = 90
ANGLE_SHOULDER_UP = 90
ANGLE_ELBOW_UP = 90
ANGLE_SHOULDER_DOWN = 20
ANGLE_ELBOW_DOWN = 130
ANGLE_CLAW_OPEN = 10
ANGLE_CLAW_CLOSE = 90

def init_arm():
    if not HAS_SERVO: return
    print("Возврат в стартовую позу...")
    servo.set(SERVO_BASE, ANGLE_BASE_CENTER)
    time.sleep(0.3)
    servo.set(SERVO_SHOULDER, ANGLE_SHOULDER_UP)
    time.sleep(0.3)
    servo.set(SERVO_ELBOW, ANGLE_ELBOW_UP)
    time.sleep(0.3)
    servo.set(SERVO_CLAW, ANGLE_CLAW_CLOSE)
    time.sleep(0.3)
    servo.set(SERVO_CAMERA_PAN, 90) # Центр камеры
    print("Поза инициализирована!")

def process_gripper(cmd):
    if not HAS_SERVO: return
    if cmd == 1:
        servo.set(SERVO_SHOULDER, ANGLE_SHOULDER_DOWN)
        time.sleep(0.2)
        servo.set(SERVO_ELBOW, ANGLE_ELBOW_DOWN)
        time.sleep(0.2)
        servo.set(SERVO_CLAW, ANGLE_CLAW_OPEN)
    elif cmd == 2:
        servo.set(SERVO_CLAW, ANGLE_CLAW_CLOSE)
        time.sleep(0.5)
        servo.set(SERVO_ELBOW, ANGLE_ELBOW_UP)
        time.sleep(0.2)
        servo.set(SERVO_SHOULDER, ANGLE_SHOULDER_UP)
    elif cmd == 3:
        init_arm()

def process_camera(yaw):
    if not HAS_SERVO: return
    # yaw от -1 до 1 -> угол от 0 до 180
    angle = 90 + (yaw * 90)
    angle = max(0, min(180, angle))
    servo.set(SERVO_CAMERA_PAN, int(angle))

# --- Главный цикл UDP сервера ---
UDP_IP = "0.0.0.0"
UDP_PORT = 10005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("============================================")
print(f"Прямой UDP-сервер запущен на порту {UDP_PORT}!")
print("Больше никаких Docker и ROS! Жду команды от Unity...")
print("============================================")

init_arm()

# --- АВАРИЙНАЯ ОСТАНОВКА ---
def emergency_stop():
    print("🛑 Аварийный стоп моторов!")
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

last_cmd_time = time.time()
WATCHDOG_TIMEOUT = 0.5

while True:
    try:
        sock.settimeout(WATCHDOG_TIMEOUT)
        data, addr = sock.recvfrom(1024)
        last_cmd_time = time.time()
        msg = json.loads(data.decode('utf-8'))
        
        cmd_type = msg.get("type", "")
        
        if cmd_type == "twist":
            process_twist(msg.get("linear", 0.0), msg.get("angular", 0.0))
        elif cmd_type == "gripper":
            process_gripper(msg.get("cmd", 0))
        elif cmd_type == "camera":
            process_camera(msg.get("yaw", 0.0))
    except socket.timeout:
        # Watchdog: нет команд → стоп
        if abs(prev_pwm_left) > 0 or abs(prev_pwm_right) > 0:
            print("⚠️ WATCHDOG: Нет команд %.1fs! СТОП!" % WATCHDOG_TIMEOUT)
            set_motors_pwm(0, 0)
    except Exception as e:
        print("Ошибка обработки:", e)
