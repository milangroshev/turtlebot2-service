#!/bin/bash

#Assemble docker image. 
echo 'Building auto-nav docker image.'
sudo docker build . -t auto-nav
