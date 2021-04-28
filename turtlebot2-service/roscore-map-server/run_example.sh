#!/bin/bash

#Assemble docker image. 
echo 'Running roscore docker image.'

sudo docker run \
        --hostname roscore-map-server \
        -it \
        --name roscore-map-server \
        --rm \
        --net host\
        -e ROS_MASTER_URI="http://192.168.55.99:11311" \
        --add-host robot01:192.168.55.7 \
        --add-host roscore-map-server:192.168.55.99 \
        --add-host amcl:192.168.55.99 \
        --add-host auto-nav:192.168.55.99 \
        --add-host drivers:192.168.55.99 \
        roscore-map-server:latest
