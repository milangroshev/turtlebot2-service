FROM ros:kinetic
MAINTAINER mgroshev@pa.uc3m.es

WORKDIR /root

RUN groupadd -g 1000 turtlebot
RUN useradd -d /home/turtlebot -s /bin/bash -m turtlebot -u 1000 -g 1000 && echo "turtlebot:turtlebot" | chpasswd && adduser turtlebot sudo

RUN \
  apt-get update && \
  apt-get install -y lsof && \
  apt-get install -y xauth && \
  apt-get install -y python-rosinstall python-rosinstall-generator && \
  apt-get install -y ros-kinetic-rviz && \
  apt-get install -y ros-kinetic-turtlebot-navigation


COPY launch/* /opt/ros/kinetic/share/rviz/launch/
COPY rviz/* /opt/ros/kinetic/share/rviz/

USER turtlebot
WORKDIR /home/turtlebot

RUN mkdir -p /home/turtlebot/scripts
COPY scripts/* /home/turtlebot/scripts/
WORKDIR /home/turtlebot/scripts/

USER root
RUN chmod +755 wait-for-ros-nodes.sh
USER turtlebot
ENV HOME /home/turtlebot
CMD bash -c "./wait-for-ros-nodes.sh"
