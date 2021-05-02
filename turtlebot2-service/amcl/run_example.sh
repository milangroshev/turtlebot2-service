#/bin/bash

#Assemble docker image. 
echo 'Running amcl docker image.'

POSE_X="2.0"
POSE_Y="2.0"
POSE_A="0.0"
SCAN_TOPIC="scan"

sudo docker run \
        --hostname amcl \
        -it \
        --name amcl \
        --rm \
        -d \
        --net host\
        -e ROS_MASTER_URI="http://127.0.0.1:11311" \
        -e POSE_X=$POSE_X \
        -e POSE_Y=$POSE_Y \
        -e POSE_A=$POSE_A \
        -e SCAN_TOPIC=$SCAN_TOPIC \
        --add-host robot01:192.168.55.7 \
        --add-host roscore-map-server:127.0.0.1 \
        --add-host amcl:127.0.0.1 \
        --add-host auto-nav:127.0.0.1 \
        --add-host drivers:127.0.0.1 \
        --add-host digital-twin:127.0.1.1 \
        amcl:latest
