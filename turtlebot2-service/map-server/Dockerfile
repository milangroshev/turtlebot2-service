FROM brain:kinetic
MAINTAINER mgroshev@pa.uc3m.es


USER docker
WORKDIR /home/docker/

CMD bash -c "source /opt/ros/kinetic/setup.bash && roslaunch turtlebot_navigation roscore_map_server.launch"

