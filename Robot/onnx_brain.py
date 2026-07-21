#!/usr/bin/env python3
"""
onnx_brain.py — standalone inference на Pi без Unity.

Загружает обученную ONNX-модель, читает сенсоры напрямую,
запускает inference и управляет моторами/серво локально.

Требования на Pi:
    pip3 install onnxruntime

Запуск:
    python3 onnx_brain.py --model /path/to/GFSX_Brain.onnx

YOLO-vision должен быть запущен отдельно (yolo_vision_node.py),
шлёт JSON по UDP на localhost:5005.
"""

import sys
import time
import json
import socket
import signal
import threading
import argparse
import math
import logging
import os
from datetime import datetime
import numpy as np

sys.path.append('/root/XiaoRGeek')

import onnxruntime as ort
import config as cfg

# ==========================================
# GPIO + ДРАЙВЕРЫ
# ==========================================
import xr_gpio as gpio

from xr_ultrasonic import Ultrasonic
us = Ultrasonic()

import xr_ultrasonic
servo = xr_ultrasonic.servo

# ==========================================
# ЛОГИРОВАНИЕ
# ==========================================
LOG_DIR = "/tmp/onnx_logs"
os.makedirs(LOG_DIR, exist_ok=True)
log_filename = os.path.join(LOG_DIR, f"brain_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

log = logging.getLogger("brain")
log.setLevel(logging.DEBUG)
_console = logging.StreamHandler()
_console.setLevel(logging.INFO)
_console.setFormatter(logging.Formatter("[%(asctime)s] %(message)s", datefmt="%H:%M:%S"))
log.addHandler(_console)

csv_file = None
csv_header = (
    "t,dt_ms,"
    "uz_raw,uz_norm,ir_l_raw,ir_r_raw,ir_g_raw,ir_l,ir_r,ir_g,"
    "yolo_vis,yolo_angle,yolo_dist,"
    "obs_0,obs_1,obs_2,obs_3,obs_4,obs_5,obs_6,obs_7,"
    "obs_8,obs_9,obs_10,obs_11,obs_12,obs_13,obs_14,"
    "act_gas,act_steer,act_cam,"
    "v_left,v_right,pwm_l,pwm_r,"
    "cam_angle,holding"
)

def init_csv():
    global csv_file
    csv_file = open(log_filename, "w")
    csv_file.write(csv_header + "\n")
    log.info(f"CSV log: {log_filename}")

def log_tick(tick_data):
    if csv_file:
        csv_file.write(",".join(f"{v:.4f}" if isinstance(v, float) else str(v) for v in tick_data) + "\n")
        csv_file.flush()

# ==========================================
# КОНСТАНТЫ
# ==========================================
INFERENCE_HZ = 10
CAMERA_SERVO_MAX_ANGLE = 90.0
OBS_SIZE = 15
MEMORY_SIZE = 256

# ==========================================
# МОТОРЫ (копия логики из unity_master_team2.py)
# ==========================================
prev_pwm_left = 0.0
prev_pwm_right = 0.0

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def stop_motors():
    gpio.digital_write(gpio.IN1, 0)
    gpio.digital_write(gpio.IN2, 0)
    gpio.digital_write(gpio.IN3, 0)
    gpio.digital_write(gpio.IN4, 0)
    gpio.ena_pwm(0)
    gpio.enb_pwm(0)

def set_motors_pwm(pwm_left, pwm_right):
    global prev_pwm_left, prev_pwm_right

    if cfg.MOTOR_INVERT_LEFT:
        pwm_left = -pwm_left
    if cfg.MOTOR_INVERT_RIGHT:
        pwm_right = -pwm_right

    if abs(pwm_left) < 0.5 and abs(pwm_right) < 0.5:
        prev_pwm_left = 0.0
        prev_pwm_right = 0.0
        stop_motors()
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
        stop_motors()
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

# ==========================================
# СЕРВО (манипулятор + камера)
# ==========================================
current_camera_angle = 90.0
is_holding = False

def init_arm():
    servo.set(cfg.SERVO_BASE, cfg.ANGLE_BASE_CENTER)
    time.sleep(0.3)
    servo.set(cfg.SERVO_SHOULDER, cfg.ANGLE_SHOULDER_UP)
    time.sleep(0.3)
    servo.set(cfg.SERVO_ELBOW, cfg.ANGLE_ELBOW_UP)
    time.sleep(0.3)
    servo.set(cfg.SERVO_CLAW, cfg.ANGLE_CLAW_OPEN)
    time.sleep(0.3)
    servo.set(cfg.SERVO_CAMERA_LOW, 90)

def gripper_grab():
    global is_holding
    servo.set(cfg.SERVO_BASE, cfg.ANGLE_BASE_CENTER)
    time.sleep(0.1)
    servo.set(cfg.SERVO_CLAW, cfg.ANGLE_CLAW_CLOSE)
    time.sleep(0.1)
    servo.set(cfg.SERVO_ELBOW, cfg.ANGLE_ELBOW_UP)
    time.sleep(0.1)
    servo.set(cfg.SERVO_SHOULDER, cfg.ANGLE_SHOULDER_UP)
    is_holding = True

def gripper_prepare():
    servo.set(cfg.SERVO_BASE, cfg.ANGLE_BASE_CENTER)
    time.sleep(0.1)
    servo.set(cfg.SERVO_SHOULDER, cfg.ANGLE_SHOULDER_DOWN)
    time.sleep(0.1)
    servo.set(cfg.SERVO_ELBOW, cfg.ANGLE_ELBOW_DOWN)
    time.sleep(0.1)
    servo.set(cfg.SERVO_CLAW, cfg.ANGLE_CLAW_OPEN)

def apply_camera(target_normalized):
    global current_camera_angle
    target_deg = 90 - (target_normalized * 90)
    target_deg = clamp(target_deg, 0, 180)

    diff = target_deg - current_camera_angle
    if abs(diff) > cfg.MAX_CAMERA_STEP:
        diff = cfg.MAX_CAMERA_STEP if diff > 0 else -cfg.MAX_CAMERA_STEP

    current_camera_angle = clamp(current_camera_angle + diff, 0, 180)
    servo.set(cfg.SERVO_CAMERA_PAN, int(current_camera_angle))

# ==========================================
# СЕНСОРЫ
# ==========================================
us_history = [100.0, 100.0, 100.0]
ir_l_history = [0, 0, 0]
ir_r_history = [0, 0, 0]
ir_grip_history = [0, 0, 0]

def read_sensors():
    global us_history
    dist_cm = us.get_distance()
    if dist_cm <= 0 or dist_cm > 500:
        dist_cm = 500.0

    us_history.pop(0)
    us_history.append(dist_cm)
    filtered_cm = sorted(us_history)[1]
    uz_norm = clamp(filtered_cm / 100.0, 0.0, 5.0) / 5.0

    ir_l_raw = 1 if gpio.digital_read(cfg.PIN_IR_LEFT) == 0 else 0
    ir_r_raw = 1 if gpio.digital_read(cfg.PIN_IR_RIGHT) == 0 else 0
    ir_g_raw = 1 if gpio.digital_read(cfg.PIN_IR_GRIPPER) == 0 else 0

    ir_l_history.pop(0)
    ir_l_history.append(ir_l_raw)
    ir_r_history.pop(0)
    ir_r_history.append(ir_r_raw)
    ir_grip_history.pop(0)
    ir_grip_history.append(ir_g_raw)

    ir_l = 1.0 if sum(ir_l_history[-3:]) >= 2 else 0.0
    ir_r = 1.0 if sum(ir_r_history[-3:]) >= 2 else 0.0
    ir_g = 1.0 if sum(ir_grip_history[-3:]) >= 2 else 0.0

    return uz_norm, ir_l, ir_r, ir_g, dist_cm, ir_l_raw, ir_r_raw, ir_g_raw

# ==========================================
# YOLO UDP ПРИЁМНИК
# ==========================================
yolo_angle = 0.0
yolo_distance = 1.0
yolo_visible = False
yolo_lock = threading.Lock()
last_yolo_time = 0.0

def yolo_listener(port=5005):
    global yolo_angle, yolo_distance, yolo_visible, last_yolo_time
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.1)

    while not _shutdown:
        try:
            data, _ = sock.recvfrom(1024)
            pkt = json.loads(data.decode())
            with yolo_lock:
                if pkt.get("sees", 0) > 0.5:
                    yolo_visible = True
                    yolo_angle = clamp(pkt.get("angle", 0.0), -1.0, 1.0)
                    yolo_distance = clamp(pkt.get("distance", 1.0), 0.0, 1.0)
                else:
                    yolo_visible = False
                    yolo_angle = 0.0
                    yolo_distance = 1.0
                last_yolo_time = time.time()
        except socket.timeout:
            pass
        except Exception:
            pass

def get_vision():
    with yolo_lock:
        if time.time() - last_yolo_time > 0.5 and last_yolo_time > 0:
            return False, 0.0, 1.0
        return yolo_visible, yolo_angle, yolo_distance

# ==========================================
# OBSERVATIONS (15 float, как в RobotBrain.cs)
# ==========================================
last_known_ball_dir = 0.0
camera_servo_normalized = 0.0
time_since_last_detection = 0.0
last_detection_time = 0.0

sensor_raw_cache = {}

def build_observations():
    global last_known_ball_dir, camera_servo_normalized
    global time_since_last_detection, last_detection_time, sensor_raw_cache

    uz, ir_l, ir_r, ir_g, uz_raw_cm, ir_l_raw, ir_r_raw, ir_g_raw = read_sensors()
    visible, v_angle, v_dist = get_vision()

    if visible:
        last_known_ball_dir = v_angle
        last_detection_time = time.time()

    if last_detection_time > 0:
        time_since_last_detection = time.time() - last_detection_time
    else:
        time_since_last_detection = 0.0

    camera_servo_normalized = clamp(
        (current_camera_angle - 90.0) / max(1.0, CAMERA_SERVO_MAX_ANGLE),
        -1.0, 1.0
    )

    obs = np.zeros(OBS_SIZE, dtype=np.float32)
    obs[0] = uz
    obs[1] = ir_l
    obs[2] = ir_r
    obs[3] = ir_g
    obs[4] = v_angle if visible else 0.0
    obs[5] = v_dist if visible else 1.0
    obs[6] = last_known_ball_dir
    obs[7] = 1.0 if visible else 0.0
    obs[8] = camera_servo_normalized
    obs[9] = 1.0 if is_holding else 0.0
    obs[10] = 0.0
    obs[11] = 0.0
    obs[12] = 0.0
    speed = (abs(prev_pwm_left) + abs(prev_pwm_right)) / 2.0 / 100.0 * cfg.MAX_SPEED_M_S
    obs[13] = speed
    obs[14] = min(time_since_last_detection, 10.0)

    sensor_raw_cache = {
        "uz_raw": uz_raw_cm,
        "ir_l_raw": ir_l_raw,
        "ir_r_raw": ir_r_raw,
        "ir_g_raw": ir_g_raw,
        "yolo_vis": visible,
        "yolo_angle": v_angle,
        "yolo_dist": v_dist,
    }

    return obs

# ==========================================
# ГЛАВНЫЙ ЦИКЛ
# ==========================================
_shutdown = False

def signal_handler(sig, frame):
    global _shutdown
    _shutdown = True

def main():
    global _shutdown, is_holding

    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, help="Path to GFSX_Brain.onnx")
    parser.add_argument("--hz", type=int, default=INFERENCE_HZ, help="Inference frequency")
    parser.add_argument("--no-yolo", action="store_true", help="Run without YOLO vision")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print(f"Loading ONNX model: {args.model}")
    session = ort.InferenceSession(args.model)
    print("Model loaded!")

    input_names = [i.name for i in session.get_inputs()]
    output_names = [o.name for o in session.get_outputs()]
    print(f"Inputs:  {input_names}")
    print(f"Outputs: {output_names}")

    memory = np.zeros((1, 1, MEMORY_SIZE), dtype=np.float32)

    if not args.no_yolo:
        yolo_thread = threading.Thread(target=yolo_listener, daemon=True)
        yolo_thread.start()
        print("YOLO UDP listener started on port 5005")

    log.info("Initializing arm...")
    init_arm()
    gripper_prepare()
    time.sleep(0.5)

    init_csv()

    tick = 1.0 / args.hz
    step = 0
    t_start = time.time()
    log.info(f"Running at {args.hz} Hz. Ctrl+C to stop.")

    try:
        while not _shutdown:
            t0 = time.time()

            obs = build_observations()
            raw = sensor_raw_cache

            feeds = {
                "obs_0": obs.reshape(1, OBS_SIZE),
                "recurrent_in": memory,
            }
            results = session.run(output_names, feeds)

            result_dict = dict(zip(output_names, results))
            actions = result_dict["deterministic_continuous_actions"][0]
            memory = result_dict["recurrent_out"].reshape(1, 1, MEMORY_SIZE)

            gas = clamp(float(actions[0]), -1.0, 1.0)
            steer = clamp(float(actions[1]), -1.0, 1.0)
            cam_target = clamp(float(actions[2]), -1.0, 1.0)

            # Автозахват по ИК клешни
            _, _, _, ir_g, _, _, _, _ = read_sensors()
            if ir_g > 0.5 and not is_holding:
                log.info("GRAB: IR gripper triggered!")
                set_motors_pwm(0, 0)
                gripper_grab()
                gas = 0.0
                steer = 0.0

            if is_holding:
                gas = 0.0
                steer = 0.0

            v_left = clamp(gas + steer * cfg.TURN_K, -cfg.MAX_LINEAR, cfg.MAX_LINEAR)
            v_right = clamp(gas - steer * cfg.TURN_K, -cfg.MAX_LINEAR, cfg.MAX_LINEAR)
            pwm_l = clamp(v_left * cfg.PWM_CONVERSION_FACTOR, -100.0, 100.0)
            pwm_r = clamp(v_right * cfg.PWM_CONVERSION_FACTOR, -100.0, 100.0)
            set_motors_pwm(pwm_l, pwm_r)

            apply_camera(cam_target)

            elapsed = time.time() - t0
            dt_ms = elapsed * 1000.0

            log_tick([
                t0 - t_start, dt_ms,
                raw.get("uz_raw", 0), obs[0],
                raw.get("ir_l_raw", 0), raw.get("ir_r_raw", 0), raw.get("ir_g_raw", 0),
                obs[1], obs[2], obs[3],
                1 if raw.get("yolo_vis") else 0, raw.get("yolo_angle", 0), raw.get("yolo_dist", 1),
                *[float(obs[i]) for i in range(OBS_SIZE)],
                gas, steer, cam_target,
                v_left, v_right, pwm_l, pwm_r,
                current_camera_angle, 1 if is_holding else 0,
            ])

            step += 1
            if step % args.hz == 0:
                log.info(
                    f"[{step:5d}] "
                    f"UZ={raw.get('uz_raw',0):5.1f}cm "
                    f"IR[L={raw.get('ir_l_raw',0)} R={raw.get('ir_r_raw',0)} G={raw.get('ir_g_raw',0)}] "
                    f"YOLO={'Y' if raw.get('yolo_vis') else 'N'} "
                    f"act=[{gas:+.2f} {steer:+.2f} {cam_target:+.2f}] "
                    f"PWM=[{pwm_l:+5.1f} {pwm_r:+5.1f}] "
                    f"dt={dt_ms:.0f}ms"
                )

            if elapsed < tick:
                time.sleep(tick - elapsed)

    finally:
        log.info("Stopping...")
        stop_motors()
        init_arm()
        if csv_file:
            csv_file.close()
        log.info(f"Log saved: {log_filename}")
        log.info("Done.")


if __name__ == "__main__":
    main()
