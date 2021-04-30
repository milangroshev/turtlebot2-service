#!/bin/bash

#Assemble docker image. 
echo 'Running roscore docker image.'

sudo docker run \
        --hostname roscore-map-server \
        -it \
        --name roscore-map-server \
        --rm \
        -d \
        --net host\
        -e ROS_MASTER_URI="http://127.0.0.1:11311" \
        --add-host robot01:127.0.0.1 \
        --add-host roscore-map-server:127.0.0.1 \
        --add-host amcl:127.0.0.1 \
        --add-host auto-nav:127.0.0.1 \
        --add-host drivers:127.0.0.1 \
        roscore-map-server:latest
