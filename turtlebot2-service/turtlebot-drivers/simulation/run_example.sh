#!/bin/bash

#Assemble docker image. 
echo 'Running auto-nav docker image.'

SOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 777 $XAUTH



sudo docker run \
        --hostname drivers \
        -it \
	-d \
        -e DISPLAY=$DISPLAY \
	-e XAUTHORITY="/home/turtlebot/.Xauthority" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $HOME/.Xauthority:/home/turtlebot/.Xauthority \
        --name drivers \
        --rm \
        --net host\
        -e ROS_MASTER_URI="http://127.0.0.1:11311" \
        -e TURTLEBOT_STAGE_WORLD_FILE="/home/turtlebot/catkin_ws/src/turtlebot_simulator/turtlebot_stage/maps/stage/twoRobotsMaze.world" \
        --add-host robot01:127.0.0.1 \
        --add-host roscore-map-server:127.0.0.1 \
        --add-host amcl:127.0.0.1 \
        --add-host auto-nav:127.0.0.1 \
        --add-host drivers:127.0.1.1 \
        --add-host digital-twin:127.0.1.1 \
	--add-host turtlebot2:127.0.1.1 \
        sim-turtlebot-drivers:latest
