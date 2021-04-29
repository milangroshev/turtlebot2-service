FROM ros:kinetic
MAINTAINER mgroshev@pa.uc3m.es

WORKDIR /root
RUN useradd -m turtlebot && echo "turtlebot:turtlebot" | chpasswd && adduser turtlebot sudo

RUN \
  apt-get update && \
  apt-get install -y lsof && \
  apt-get install -y python-rosinstall python-rosinstall-generator && \
  apt-get install -y ros-kinetic-turtlebot-bringup && \
  apt-get install -y ros-kinetic-joint-state-publisher


COPY launch/turtlebot_robot.launch /opt/ros/kinetic/share/turtlebot_bringup/launch/

USER turtlebot
WORKDIR /home/turtlebot

RUN mkdir -p /home/turtlebot/scripts
COPY scripts/* /home/turtlebot/scripts/
WORKDIR /home/turtlebot/scripts/

USER root
RUN chmod +755 wait-for-ros-nodes.sh


USER turtlebot
CMD bash -c "./wait-for-ros-nodes.sh"
