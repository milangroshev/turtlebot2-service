#!/bin/bash

#Assemble docker image. 
echo 'Building sim-turtlebot-drivers docker image.'
docker build . -t sim-stage-ros
