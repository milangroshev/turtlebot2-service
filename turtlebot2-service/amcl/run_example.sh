#!/bin/bash

#Assemble docker image. 
echo 'Running amcl docker image.'

sudo docker run \
        --hostname amcl \
        -it \
        --name amcl \
        --rm \
        --net host\
        -e ROS_MASTER_URI="http://192.168.55.99:11311" \
        --add-host robot01:192.168.55.7 \
        --add-host roscore-map-server:192.168.55.99 \
        --add-host amcl:192.168.55.99 \
        --add-host auto-nav:192.168.55.99 \
        amcl:latest
