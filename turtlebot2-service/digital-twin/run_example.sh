#!/bin/bash

#Assemble docker image. 
echo 'Running amcl docker image.'

SOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 777 $XAUTH


sudo docker run \
        --hostname digital-twin \
        -it \
        -d \
        -e DISPLAY="digital-twin:10.0" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $HOME/.Xauthority:/home/turtlebot/.Xauthority \
        --name digital-twin \
        --rm \
        --net host\
        -e ROS_MASTER_URI="http://127.0.0.1:11311" \
        --add-host robot01:192.168.55.7 \
        --add-host roscore-map-server:127.0.0.1 \
        --add-host amcl:127.0.0.1 \
        --add-host auto-nav:127.0.0.1 \
        --add-host drivers:127.0.0.1 \
        --add-host digital-twin:127.0.1.1 \
        digital-twin:latest
