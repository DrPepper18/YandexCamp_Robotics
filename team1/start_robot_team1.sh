#!/bin/bash

# Скрипт автоматического запуска ROS-окружения на роботе
# Запускается НА Raspberry Pi

# Отключаем спящий режим WiFi для стабильной связи
sudo iw dev wlan0 set power_save off || true

CONTAINER_NAME="xiao_ros_brain"

echo "=== [1/4] Подготовка контейнера Docker ==="
# Удаляем старый контейнер (даже если он завис или работает некорректно)
echo "Очистка старого контейнера $CONTAINER_NAME..."
docker rm -f $CONTAINER_NAME 2>/dev/null || true

# Создаем абсолютно чистый контейнер с нуля!
# --pid=host ДОБАВЛЕН: без него у контейнера СВОЙ отдельный PID namespace, даже
# несмотря на --network host (тот только шарит СЕТЕВОЙ стек, порты видны, но
# /proc - нет). Если порт 2001/2002/2005 держит процесс НА САМОМ ХОСТЕ (а не
# внутри старого контейнера) - например, висящий сервис оригинального
# приложения XiaoR - force_free_port() в xr_config.py физически не может его
# ни увидеть, ни убить: он сканирует /proc, а из контейнера без --pid=host
# виден только /proc самого контейнера. С --pid=host /proc общий с хостом -
# force_free_port() видит и может убить любой процесс, что бы порт ни держало.
echo "Создаем чистый контейнер из образа..."
docker run -dt --name $CONTAINER_NAME --network host --pid=host --privileged -v /dev:/dev ros_noetic_hardware_v2 bash

# Ждем 2 секунды, чтобы демон Докера окончательно его поднял
sleep 2

echo "Контейнер готов! Обновление скриптов..."

echo "Обновление единого мастер-скрипта в контейнере..."
# Получаем директорию текущего скрипта
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
docker cp "$SCRIPT_DIR/unity_master_team1.py" $CONTAINER_NAME:/root/unity_master_team1.py
docker cp "$SCRIPT_DIR/unity_gripper_ir_team1.py" $CONTAINER_NAME:/root/unity_gripper_ir_team1.py

# Копируем наш чистый Питон-заменитель smbus прямо к драйверам XiaoRGeek
if [ -f "$SCRIPT_DIR/smbus.py" ]; then
    docker cp "$SCRIPT_DIR/smbus.py" $CONTAINER_NAME:/root/XiaoRGeek/smbus.py
fi

# Заглушка для xr_car_light (без неё xr_ultrasonic не загрузится!)
if [ -f "$SCRIPT_DIR/xr_car_light.py" ]; then
    docker cp "$SCRIPT_DIR/xr_car_light.py" $CONTAINER_NAME:/root/XiaoRGeek/xr_car_light.py
fi

# Заглушка для xr_music (без неё xr_ultrasonic также выдаст ошибку!)
if [ -f "$SCRIPT_DIR/xr_music.py" ]; then
    docker cp "$SCRIPT_DIR/xr_music.py" $CONTAINER_NAME:/root/XiaoRGeek/xr_music.py
fi

# Патч xr_config: bind() портов BT/TCP-пульта (2001/2002/2005) обёрнут в
# try/except + force_free_port(), чтобы "зависший" процесс не ронял импорт
# xr_ultrasonic/xr_servo и, как следствие, инициализацию джойнтов (init_arm())
if [ -f "$SCRIPT_DIR/xr_config.py" ]; then
    docker cp "$SCRIPT_DIR/xr_config.py" $CONTAINER_NAME:/root/XiaoRGeek/xr_config.py
    # Самопроверка: убеждаемся, что патч реально попал в контейнер, а не осталась
    # старая версия (именно это и произошло в прошлый раз - в логе была ошибка
    # без try/except на строке ~102, то есть контейнер работал со СТАРЫМ файлом)
    if docker exec $CONTAINER_NAME grep -q "force_free_port" /root/XiaoRGeek/xr_config.py; then
        echo "  [OK] xr_config.py с патчем force_free_port подтверждён внутри контейнера"
    else
        echo "  [!!] ВНИМАНИЕ: force_free_port НЕ найден в /root/XiaoRGeek/xr_config.py внутри контейнера - патч не применился!"
    fi
fi

echo "=== [1.5/4] Копирование видео-стримера ==="
docker cp "$SCRIPT_DIR/camera_stream_team1.py" $CONTAINER_NAME:/root/camera_stream_team1.py
# Запуск внутри докера отключен, так как там нет библиотеки cv2.
# Стрим запускается на хосте через ./camera_stream_team1
# docker exec -d $CONTAINER_NAME python3 /root/camera_stream_team1.py

echo "=== [2/4] Запуск roscore ==="
# Запускаем в фоне (-d) вместо интерактивного (-it)
docker exec -d $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && roscore"

echo "Ожидание запуска roscore (3 сек)..."
sleep 3

echo "=== [3/4] Запуск ROS-TCP Endpoint ==="
docker exec -d $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && rosrun ros_tcp_endpoint default_server_endpoint.py --tcp_ip 0.0.0.0 --tcp_port 10000"

echo "Ожидание запуска Endpoint (2 сек)..."
sleep 2

echo "=== [4/4] Запуск ЕДИНОГО Узла Робота (unity_master_team1.py) ==="
docker exec -d $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && python3 -u /root/unity_master_team1.py > /tmp/master.log 2>&1"

echo "=== [4.5/4] Запуск Узла ИК-клешни (unity_gripper_ir_team1.py) ==="
docker exec -d $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && python3 -u /root/unity_gripper_ir_team1.py > /tmp/gripper_ir.log 2>&1"

echo "=== ВСЕ СЛУЖБЫ ЗАПУЩЕНЫ В ФОНОВОМ РЕЖИМЕ ДЛЯ TEAM1 ==="
echo "Проверить логи мастер-скрипта  : docker exec xiao_ros_brain cat /tmp/master.log"
echo "Проверить логи клешни (ИК)     : docker exec xiao_ros_brain cat /tmp/gripper_ir.log"
