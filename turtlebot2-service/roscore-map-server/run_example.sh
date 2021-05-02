#!/bin/bash

#Assemble docker image. 
echo 'Running roscore docker image.'

MAP="/opt/ros/kinetic/share/map_server/maze.yaml"

sudo docker run \
        --hostname roscore-map-server \
        -it \
        --name roscore-map-server \
        --rm \
        -d \
        --net host\
        -e ROS_MASTER_URI="http://127.0.0.1:11311" \
        -e TURTLEBOT_STAGE_MAP_FILE=$MAP \
        --add-host robot01:192.168.55.7 \
        --add-host roscore-map-server:127.0.0.1 \
        --add-host amcl:127.0.0.1 \
        --add-host auto-nav:127.0.0.1 \
        --add-host drivers:127.0.0.1 \
        --add-host digital-twin:127.0.1.1 \
        roscore-map-server:latest
