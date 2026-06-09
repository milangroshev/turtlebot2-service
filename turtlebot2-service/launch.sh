#!/bin/bash

# Launches the full turtlebot2 simulation stack in dependency order:
#   roscore-map-server → sim-stage-ros + sim-turtlebot-drivers → amcl → auto-nav → digital-twin

ROS_MASTER="http://127.0.0.1:11311"
ROBOT_NS="robot_0"

# --- X11 setup for GUI containers ---
touch "${HOME}/.Xauthority"
XAUTH=/tmp/.docker.xauth
touch "$XAUTH"
xauth nlist "$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - 2>/dev/null
chmod 777 "$XAUTH"

X11_ARGS=(
    -e DISPLAY="$DISPLAY"
    -e XAUTHORITY="/home/turtlebot/.Xauthority"
    -v /tmp/.X11-unix:/tmp/.X11-unix
    -v "${HOME}/.Xauthority:/home/turtlebot/.Xauthority"
)

# --- Cleanup on exit ---
cleanup() {
    trap - EXIT INT TERM
    echo ""
    echo "Stopping all containers..."
    docker rm -f roscore-map-server stage-ros drivers amcl auto-nav digital-twin 2>/dev/null || true
    echo "Done."
    exit 0
}
trap cleanup EXIT INT TERM

# --- Wait until a ROS node is visible on the master ---
wait_for_node() {
    local node="$1"
    local timeout="${2:-90}"
    local elapsed=0
    echo -n "  Waiting for ${node}..."
    while ! docker exec roscore-map-server bash -c \
        "source /opt/ros/kinetic/setup.bash && rosnode list 2>/dev/null | grep -qx '${node}'" 2>/dev/null; do
        sleep 2
        elapsed=$((elapsed + 2))
        if [ "$elapsed" -ge "$timeout" ]; then
            echo " TIMEOUT (${timeout}s)"
            exit 1
        fi
        echo -n "."
    done
    echo " ready"
}

# --- Clean up leftover containers from a previous run ---
docker rm -f roscore-map-server stage-ros drivers amcl auto-nav digital-twin 2>/dev/null || true

echo "========================================"
echo "  Launching turtlebot2 simulation"
echo "========================================"

# 1. roscore + map_server
echo "[1/6] roscore-map-server"
docker run \
    --hostname roscore-map-server \
    -d --name roscore-map-server --rm --net host \
    -e ROS_MASTER_URI="$ROS_MASTER" \
    -e TURTLEBOT_STAGE_MAP_FILE="/opt/ros/kinetic/share/map_server/maze.yaml" \
    --add-host robot01:192.168.55.7 \
    --add-host roscore-map-server:127.0.0.1 \
    --add-host amcl:127.0.0.1 \
    --add-host auto-nav:127.0.0.1 \
    --add-host drivers:127.0.0.1 \
    --add-host digital-twin:127.0.1.1 \
    --add-host stage-ros:127.0.1.1 \
    roscore-map-server:latest > /dev/null

wait_for_node /map_server

# 2. Stage simulator
echo "[2/6] sim-stage-ros"
docker run \
    --hostname stage-ros \
    -d --name stage-ros --rm --net host \
    "${X11_ARGS[@]}" \
    -e ROS_MASTER_URI="$ROS_MASTER" \
    -e TURTLEBOT_STAGE_WORLD_FILE="/home/turtlebot/catkin_ws/src/turtlebot_simulator/turtlebot_stage/maps/stage/room_with_static_objects.world" \
    --add-host robot01:127.0.0.1 \
    --add-host roscore-map-server:127.0.0.1 \
    --add-host amcl:127.0.0.1 \
    --add-host auto-nav:127.0.0.1 \
    --add-host stage-ros:127.0.1.1 \
    --add-host drivers:127.0.1.1 \
    --add-host digital-twin:127.0.1.1 \
    --add-host turtlebot2:127.0.1.1 \
    sim-stage-ros:latest > /dev/null

# 3. Turtlebot drivers (both 2 and 3 wait for /map_server internally)
echo "[3/6] sim-turtlebot-drivers"
docker run \
    --hostname drivers \
    -d --name drivers --rm --net host \
    "${X11_ARGS[@]}" \
    -e ROS_MASTER_URI="$ROS_MASTER" \
    -e TURTLEBOT_STAGE_WORLD_FILE="/home/turtlebot/catkin_ws/src/turtlebot_simulator/turtlebot_stage/maps/stage/twoRobotsMaze.world" \
    --add-host robot01:127.0.0.1 \
    --add-host roscore-map-server:127.0.0.1 \
    --add-host amcl:127.0.0.1 \
    --add-host auto-nav:127.0.0.1 \
    --add-host drivers:127.0.1.1 \
    --add-host digital-twin:127.0.1.1 \
    --add-host turtlebot2:127.0.1.1 \
    --add-host stage-ros:127.0.1.1 \
    sim-turtlebot-drivers:latest > /dev/null

wait_for_node /stageros
wait_for_node "/${ROBOT_NS}/cmd_vel_mux"

# 4. AMCL localization
echo "[4/6] amcl"
docker run \
    --hostname amcl \
    -d --name amcl --rm --net host \
    -e ROS_MASTER_URI="$ROS_MASTER" \
    -e POSE_X="2.0" \
    -e POSE_Y="2.0" \
    -e POSE_A="0.0" \
    -e SCAN_TOPIC="scan" \
    -e ROBOT_NS="$ROBOT_NS" \
    -e ODOM_FRAME="${ROBOT_NS}/odom" \
    -e BASE_FRAME="${ROBOT_NS}/base_footprint" \
    --add-host robot01:192.168.55.7 \
    --add-host roscore-map-server:127.0.0.1 \
    --add-host amcl:127.0.0.1 \
    --add-host auto-nav:127.0.0.1 \
    --add-host drivers:127.0.0.1 \
    --add-host digital-twin:127.0.1.1 \
    --add-host stage-ros:127.0.1.1 \
    amcl:latest > /dev/null

wait_for_node "/${ROBOT_NS}/amcl"

# 5. Navigation stack
echo "[5/6] auto-nav"
docker run \
    --hostname auto-nav \
    -d --name auto-nav --rm --net host \
    -e ROS_MASTER_URI="$ROS_MASTER" \
    -e SCAN_TOPIC="scan" \
    -e ROBOT_NS="$ROBOT_NS" \
    --add-host robot01:127.0.0.1 \
    --add-host roscore-map-server:127.0.0.1 \
    --add-host amcl:127.0.0.1 \
    --add-host auto-nav:127.0.0.1 \
    --add-host drivers:127.0.0.1 \
    --add-host digital-twin:127.0.1.1 \
    --add-host stage-ros:127.0.1.1 \
    auto-nav:latest > /dev/null

wait_for_node "/${ROBOT_NS}/move_base"

# 6. Digital twin / map navigation
echo "[6/6] digital-twin"
docker run \
    --hostname digital-twin \
    -d --name digital-twin --rm --net host \
    "${X11_ARGS[@]}" \
    -e ROS_MASTER_URI="$ROS_MASTER" \
    --add-host robot01:127.0.0.1 \
    --add-host roscore-map-server:127.0.0.1 \
    --add-host amcl:127.0.0.1 \
    --add-host auto-nav:127.0.0.1 \
    --add-host drivers:127.0.0.1 \
    --add-host digital-twin:127.0.1.1 \
    --add-host stage-ros:127.0.1.1 \
    digital-twin:latest > /dev/null

echo ""
echo "========================================"
echo "  Simulation running"
echo "========================================"
docker exec roscore-map-server bash -c \
    "source /opt/ros/kinetic/setup.bash && rosnode list" 2>&1
echo ""
echo "Press Ctrl+C to stop all containers."

while true; do
    sleep 5
    for name in roscore-map-server stage-ros drivers amcl auto-nav digital-twin; do
        if ! docker ps --format "{{.Names}}" 2>/dev/null | grep -qx "$name"; then
            echo "WARNING: container '$name' has stopped unexpectedly."
        fi
    done
done
