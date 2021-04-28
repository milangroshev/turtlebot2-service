#!/bin/bash

#Assemble docker image. 
echo 'Building turtlebot-drivers docker image.'
sudo docker build . -t turtlebot-drivers
