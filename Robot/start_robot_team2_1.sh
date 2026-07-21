#!/bin/bash

# Скрипт запуска ТОЛЬКО демо-скрипта "клешня + проезд 1м туда-обратно"
# ROS / Unity стек НЕ запускается (roscore, ros_tcp_endpoint, unity_master, gripper_ir - отключены).
# Запускается НА Raspberry Pi.

# Отключаем спящий режим WiFi для стабильной связи
sudo iw dev wlan0 set power_save off || true

CONTAINER_NAME="xiao_ros_brain"

echo "=== [1/3] Подготовка контейнера Docker ==="
echo "Очистка старого контейнера $CONTAINER_NAME..."
docker rm -f $CONTAINER_NAME 2>/dev/null || true

echo "Создаем чистый контейнер из образа..."
docker run -dt --name $CONTAINER_NAME --network host --privileged -v /dev:/dev ros_noetic_hardware_v2 bash

# Ждем 2 секунды, чтобы демон Докера окончательно его поднял
sleep 2

echo "Контейнер готов! Копирование скрипта..."

# Получаем директорию текущего скрипта
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo "=== [2/3] Копирование демо-скрипта и заглушек драйверов ==="
docker cp "$SCRIPT_DIR/claw_open_drive_1m_back.py" $CONTAINER_NAME:/root/claw_open_drive_1m_back.py

# Копируем наш чистый Питон-заменитель smbus прямо к драйверам XiaoRGeek
# (нужен для xr_servo -> xr_i2c)
if [ -f "$SCRIPT_DIR/smbus.py" ]; then
    docker cp "$SCRIPT_DIR/smbus.py" $CONTAINER_NAME:/root/XiaoRGeek/smbus.py
fi

echo "=== [3/3] Запуск демо-скрипта (клешня -> 1м вперёд -> стоп -> назад -> стоп) ==="
# Запускаем в переднем плане (без -d), чтобы видеть вывод и дождаться завершения.
docker exec $CONTAINER_NAME bash -c "python3 -u /root/claw_open_drive_1m_back.py"

echo "=== ГОТОВО: демо-скрипт завершён ==="
echo "Логи можно посмотреть повторно: docker exec $CONTAINER_NAME cat /tmp/demo.log (если перенаправляли вывод)"
