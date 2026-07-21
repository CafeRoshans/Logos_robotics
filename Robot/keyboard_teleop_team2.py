#!/usr/bin/env python3
"""
keyboard_teleop_team2.py — клавиатурное танковое управление роботом через ROS-топики.

Публикует в те же топики, что и Unity через ROS-TCP endpoint, поэтому
работает ПАРАЛЛЕЛЬНО с unity_master_team2.py — менять его не нужно.
Конфликт источников команд снимается watchdog'ом в unity_master_team2.py
(стоп через 0.5с без новых команд) — активен тот, кто писал последним.

Топик /cmd_vel принимает только Twist (linear + angular), а не скорости
гусениц напрямую, поэтому раздельные W/S и E/D здесь пересчитываются в
linear.x / angular.z так, чтобы на стороне unity_master_team2.py после
его формулы v_left = lin_x + ang_z*TURN_K / v_right = lin_x - ang_z*TURN_K
получились именно те скорости гусениц, которые запросила клавиатура.
Из-за этого MASTER_TURN_K в конфиге ниже должен совпадать с TURN_K в
unity_master_team2.vel_callback — если поменяют там, поменять и здесь.

Управление:
    W / S       — левая гусеница вперёд / назад
    E / D       — правая гусеница вперёд / назад
    ←/→         — плавный поворот камеры
    G / F       — опустить руку и открыть клешню / захватить и поднять
    R           — стартовая поза манипулятора
    O           — только открыть клешню (рука не двигается)
    space / X   — принудительный стоп обеих гусениц
    Q           — выход
"""

import curses
import time
import sys

sys.path.append('/root/XiaoRGeek')
import config as cfg

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32


# ======================================================================
# КОНФИГУРАЦИЯ
# ======================================================================
class KeyConfig:
    TOPIC_CMD_VEL = "/cmd_vel"
    TOPIC_CMD_GRIPPER = "/cmd_gripper"
    TOPIC_CMD_CAMERA = "/cmd_camera_pan"

    MAX_LINEAR = cfg.MAX_LINEAR
    MASTER_TURN_K = cfg.TURN_K

    # --- Камера (нормализованная ось -1..1, как ждёт camera_callback) ---
    CAMERA_STEP = 0.08      # шаг за один тик при удержании стрелки
    CAMERA_MIN = -1.0
    CAMERA_MAX = 1.0

    # --- Тайминги ---
    PUBLISH_HZ = 20
    DIR_TIMEOUT = 0.2       # сек без нажатия — автостоп соответствующей гусеницы

    # --- Коды команд клешни, см. unity_master_team2.gripper_callback ---
    GRIPPER_PREPARE = 1     # опустить руку, открыть
    GRIPPER_GRAB = 2        # закрыть, поднять
    GRIPPER_INIT = 3        # стартовая поза
    GRIPPER_RELEASE = 4     # только открыть, рука не двигается

    # --- Раскладка клавиш ---
    KEY_FORWARD = "w"
    KEY_BACKWARD = "s"
    KEY_LEFT = "a"
    KEY_RIGHT = "d"
    
    KEYS_CAMERA = {                        # curses-коды стрелок
        curses.KEY_LEFT: -1,
        curses.KEY_RIGHT: 1,
    }
    KEYS_GRIPPER = {
        "g": GRIPPER_PREPARE,
        "f": GRIPPER_GRAB,
        "r": GRIPPER_INIT,
        "o": GRIPPER_RELEASE,
    }
    KEYS_STOP = (" ", "x", "X")
    KEYS_QUIT = ("q", "Q")


# ======================================================================
# СОСТОЯНИЕ И ЛОГИКА
# ======================================================================
class State:
    def __init__(self):
        self.left_dir = 0
        self.right_dir = 0
        self.last_left_key_time = 0.0
        self.last_right_key_time = 0.0
        self.camera_yaw = 0.0
        self.gripper_last_cmd: int | None = None


class KeyboardTeleop:
    def __init__(self, cfg: KeyConfig):
        self.cfg = cfg
        self.state = State()
        self.vel_pub = rospy.Publisher(cfg.TOPIC_CMD_VEL, Twist, queue_size=1)
        self.gripper_pub = rospy.Publisher(cfg.TOPIC_CMD_GRIPPER, Int32, queue_size=1)
        self.camera_pub = rospy.Publisher(cfg.TOPIC_CMD_CAMERA, Float32, queue_size=1)

    def handle_key(self, ch: int, now: float) -> bool:
        """Обрабатывает один код клавиши. Возвращает False при выходе."""
        cfg, st = self.cfg, self.state

        if ch == -1:
            return True

        key = chr(ch) if 0 <= ch < 256 else None

        if key in cfg.KEYS_QUIT:
            st.left_dir = 0
            st.right_dir = 0
            return False

        if key in cfg.KEYS_STOP:
            st.left_dir = 0
            st.right_dir = 0
            st.last_left_key_time = 0.0
            st.last_right_key_time = 0.0
        elif key == cfg.KEY_FORWARD:
            st.left_dir = 1
            st.right_dir = 1
            st.last_left_key_time = now
            st.last_right_key_time = now  
        elif key == cfg.KEY_BACKWARD:
            st.left_dir = -1
            st.right_dir = -1
            st.last_left_key_time = now
            st.last_right_key_time = now
        elif key == cfg.KEY_LEFT:
            st.left_dir = -1
            st.right_dir = 1
            st.last_left_key_time = now
            st.last_right_key_time = now
        elif key == cfg.KEY_RIGHT:
            st.left_dir = 1
            st.right_dir = -1
            st.last_left_key_time = now
            st.last_right_key_time = now
        
        elif ch in cfg.KEYS_CAMERA:
            delta = cfg.KEYS_CAMERA[ch] * cfg.CAMERA_STEP
            st.camera_yaw = max(cfg.CAMERA_MIN, min(cfg.CAMERA_MAX, st.camera_yaw + delta))
        elif key in cfg.KEYS_GRIPPER:
            cmd = cfg.KEYS_GRIPPER[key]
            self.gripper_pub.publish(Int32(cmd))
            st.gripper_last_cmd = cmd

        return True

    def apply_watchdog(self, now: float) -> None:
        """Каждая гусеница останавливается независимо, если её клавиша не
        нажималась дольше DIR_TIMEOUT — отпустил W/S, левая встала, ED
        при этом продолжает работать, если E/D всё ещё удерживаются."""
        st, cfg = self.state, self.cfg
        if st.left_dir and now - st.last_left_key_time > cfg.DIR_TIMEOUT:
            st.left_dir = 0
            st.last_left_key_time = 0.0
        if st.right_dir and now - st.last_right_key_time > cfg.DIR_TIMEOUT:
            st.right_dir = 0
            st.last_right_key_time = 0.0

    def publish_state(self) -> None:
        """Пересчитывает независимые left_dir/right_dir в Twist так, чтобы
        после дифференциальной формулы unity_master_team2.py получились
        именно запрошенные скорости гусениц (см. вывод в докстринге)."""
        st, cfg = self.state, self.cfg
        lin_x = (st.left_dir + st.right_dir) / 2.0 * cfg.MAX_LINEAR
        ang_z = (st.left_dir - st.right_dir) * cfg.MAX_LINEAR / (2.0 * cfg.MASTER_TURN_K)

        twist = Twist()
        twist.linear.x = lin_x
        twist.angular.z = ang_z
        self.vel_pub.publish(twist)
        self.camera_pub.publish(Float32(st.camera_yaw))

    def status_lines(self) -> list:
        st = self.state
        return [
            "Клавиатурный пульт team2 — q: выход",
            "W/S левая | E/D правая | \u2190/\u2192 камера | g/f/r/o клешня | space/x стоп",
            f"left={st.left_dir:+d} right={st.right_dir:+d} "
            f"camera_yaw={st.camera_yaw:+.2f} gripper_cmd={st.gripper_last_cmd}",
        ]


# ======================================================================
# ТОЧКА ВХОДА
# ======================================================================
def main(stdscr) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    rospy.init_node("keyboard_teleop_team2", anonymous=True)
    node = KeyboardTeleop(KeyConfig())

    tick = 1.0 / KeyConfig.PUBLISH_HZ
    running = True

    try:
        while running and not rospy.is_shutdown():
            now = time.time()

            running = node.handle_key(stdscr.getch(), now)
            node.apply_watchdog(now)
            node.publish_state()

            stdscr.clear()
            for i, line in enumerate(node.status_lines()):
                stdscr.addstr(i, 0, line)
            stdscr.refresh()

            time.sleep(tick)
    except KeyboardInterrupt:
        pass  # Ctrl+C
    finally:
        # гарантированный стоп гусениц при любом выходе из цикла
        node.state.left_dir = 0
        node.state.right_dir = 0
        node.publish_state()


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass