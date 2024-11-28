#!/bin/bash

# Assemble docker image.
echo 'Running auto-nav docker image.'

# Variables de entorno para auto-nav
SCAN_TOPIC="scan"
ROBOT_NS="robot_0"

# Directorio local para guardar los datos generados por el data_collector
DATA_COLLECTOR_HOST_DIR="$(pwd)/data_collector"
DATA_COLLECTOR_CONTAINER_DIR="/home/ros/data_collector"

# Crear la carpeta de datos en el host si no existe
mkdir -p "$DATA_COLLECTOR_HOST_DIR"

# Ejecutar el contenedor auto-nav con data_collector como un proceso separado
docker run \
    --hostname auto-nav \
    -it \
    --name auto-nav \
    --rm \
    --net host \
    -e ROS_MASTER_URI="http://127.0.0.1:11311" \
    -e SCAN_TOPIC=$SCAN_TOPIC \
    -e ROBOT_NS=$ROBOT_NS \
    --add-host robot01:127.0.0.1 \
    --add-host roscore-map-server:127.0.0.1 \
    --add-host amcl:127.0.0.1 \
    --add-host auto-nav:127.0.0.1 \
    --add-host drivers:127.0.0.1 \
    --add-host digital-twin:127.0.1.1 \
    --add-host stage-ros:127.0.1.1 \
    -v "$DATA_COLLECTOR_HOST_DIR:$DATA_COLLECTOR_CONTAINER_DIR" \
    auto-nav:latest \
    bash -c "
    source /opt/ros/kinetic/setup.bash &&
    python $DATA_COLLECTOR_CONTAINER_DIR/data_collector.py &  # Inicia el data_collector en segundo plano
    roslaunch turtlebot_navigation auto_nav.launch            # Comando principal del contenedor
    "
