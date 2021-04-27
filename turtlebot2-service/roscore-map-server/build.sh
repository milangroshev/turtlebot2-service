#!/bin/bash

#Assemble docker image. 
echo 'Building roscore-map-server docker image.'
sudo docker build . -t roscore-map-server
