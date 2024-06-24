#!/bin/bash

#Assemble docker image. 
echo 'Building sim-turtlebot-drivers docker image.'
sudo docker build . -t sim-turtlebot-drivers
