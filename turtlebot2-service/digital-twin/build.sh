#!/bin/bash

#Assemble docker image. 
echo 'Building amcl docker image.'
sudo docker build . -t amcl
