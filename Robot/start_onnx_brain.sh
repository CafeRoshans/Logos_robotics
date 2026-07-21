#!/bin/bash

# Запуск локального inference на Pi (без Unity, без onnxruntime)
# Используется numpy_brain.py — чистый numpy forward pass
# yolo_vision_node.py запускается на хосте отдельно

sudo iw dev wlan0 set power_save off || true

CONTAINER_NAME="xiao_ros_brain"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo "=== [1/3] Подготовка контейнера Docker ==="
docker rm -f $CONTAINER_NAME 2>/dev/null || true
docker run -dt --name $CONTAINER_NAME --network host --privileged -v /dev:/dev ros_noetic_hardware_v2 bash
sleep 2

echo "=== [2/3] Копирование файлов ==="
docker cp "$SCRIPT_DIR/config.py"         $CONTAINER_NAME:/root/config.py
docker cp "$SCRIPT_DIR/xr_gpio.py"        $CONTAINER_NAME:/root/XiaoRGeek/xr_gpio.py
docker cp "$SCRIPT_DIR/numpy_brain.py"    $CONTAINER_NAME:/root/numpy_brain.py
docker cp "$SCRIPT_DIR/brain_weights.npz" $CONTAINER_NAME:/root/brain_weights.npz

# Заглушки для драйверов
if [ -f "$SCRIPT_DIR/smbus.py" ]; then
    docker cp "$SCRIPT_DIR/smbus.py" $CONTAINER_NAME:/root/XiaoRGeek/smbus.py
fi
if [ -f "$SCRIPT_DIR/xr_car_light.py" ]; then
    docker cp "$SCRIPT_DIR/xr_car_light.py" $CONTAINER_NAME:/root/XiaoRGeek/xr_car_light.py
fi
if [ -f "$SCRIPT_DIR/xr_music.py" ]; then
    docker cp "$SCRIPT_DIR/xr_music.py" $CONTAINER_NAME:/root/XiaoRGeek/xr_music.py
fi

echo "=== [3/3] Запуск numpy_brain.py ==="
docker exec -it $CONTAINER_NAME python3 -u /root/numpy_brain.py --weights /root/brain_weights.npz
