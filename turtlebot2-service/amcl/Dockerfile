FROM ros:kinetic
MAINTAINER mgroshev@pa.uc3m.es

WORKDIR /root
RUN useradd -ms /bin/bash turtlebot

RUN \
  apt-get update && \
  apt-get install -y lsof && \
  apt-get install -y python-rosinstall python-rosinstall-generator && \
  apt-get install -y ros-kinetic-amcl


COPY launch/* /opt/ros/kinetic/share/amcl/launch/


USER turtlebot
WORKDIR /home/turtlebot

RUN mkdir -p /home/turtlebot/scripts
COPY scripts/* /home/turtlebot/scripts/
WORKDIR /home/turtlebot/scripts/

USER root
RUN chmod +755 wait-for-ros-nodes*
USER turtlebot

CMD bash -c "./wait-for-ros-nodes-sim.sh"
