#!/bin/bash

#Assemble docker image. 
echo 'Running auto-nav docker image.'

SCAN_TOPIC="scan"
ROBOT_NS="robot_0"


sudo docker run \
        --hostname auto-nav \
        -it \
        -d \
        --name auto-nav \
        --rm \
        --net host\
        -e ROS_MASTER_URI="http://127.0.0.1:11311" \
        -e SCAN_TOPIC=$SCAN_TOPIC \
        -e ROBOT_NS=$ROBOT_NS \
        --add-host robot01:192.168.55.7 \
        --add-host roscore-map-server:127.0.0.1 \
        --add-host amcl:127.0.0.1 \
        --add-host auto-nav:127.0.0.1 \
        --add-host drivers:127.0.0.1 \
        --add-host digital-twin:127.0.1.1 \
        auto-nav:latest
