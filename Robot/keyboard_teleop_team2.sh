#!/bin/bash
# Копирует и запускает keyboard_teleop_team2.py в уже работающем контейнере.
# Предполагает, что робот уже поднят через start_robot_team2.sh (roscore и
# unity_master_team2.py запущены) — здесь это не дублируется.

set -e

CONTAINER_NAME="xiao_ros_brain"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "Контейнер $CONTAINER_NAME не запущен. Сначала выполните start_robot_team2.sh"
    exit 1
fi

docker cp "$SCRIPT_DIR/config.py" $CONTAINER_NAME:/root/config.py
docker cp "$SCRIPT_DIR/keyboard_teleop_team2.py" $CONTAINER_NAME:/root/keyboard_teleop_team2.py

exec docker exec -it $CONTAINER_NAME bash -c \
    "source /opt/ros/noetic/setup.bash && python3 /root/keyboard_teleop_team2.py"