FROM ros:kinetic
MAINTAINER mgroshev@pa.uc3m.es

WORKDIR /root
RUN useradd -ms /bin/bash turtlebot

RUN \
  apt-get update && \
  apt-get install -y lsof && \
  apt-get install -y python-rosinstall python-rosinstall-generator && \
  apt-get install -y ros-kinetic-map-server


COPY maps/* /opt/ros/kinetic/share/map_server/
COPY launch/* /opt/ros/kinetic/share/map_server/


USER turtlebot
WORKDIR /home/turtlebot/

CMD bash -c "source /opt/ros/kinetic/setup.bash && roslaunch map_server roscore_map_server.launch --screen"
