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
  apt-get install -y ros-kinetic-turtlebot-bringup && \
  apt-get install -y ros-kinetic-joint-state-publisher && \
  apt-get install -y ros-kinetic-stage


COPY launch/turtlebot_robot.launch /opt/ros/kinetic/share/turtlebot_bringup/launch/

USER turtlebot
WORKDIR /home/turtlebot


RUN mkdir -p catkin_ws/src && cd catkin_ws/src && \
    git clone https://github.com/turtlebot/turtlebot_simulator.git && \
    git clone https://github.com/ros-simulation/stage_ros.git

RUN cd catkin_ws && bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"

RUN mkdir -p /home/turtlebot/scripts
COPY scripts/* /home/turtlebot/scripts/
COPY worlds/* /home/turtlebot/catkin_ws/src/turtlebot_simulator/turtlebot_stage/maps/stage/

WORKDIR /home/turtlebot/scripts/

USER root
RUN chmod +755 wait-for-ros-nodes.sh

RUN rm -rf /opt/ros/kinetic/share/kobuki_node/param/base.yaml
COPY param/* /opt/ros/kinetic/share/kobuki_node/param/

USER turtlebot
ENV HOME /home/turtlebot
#CMD bash -c "source /opt/ros/kinetic/setup.bash; source catkin_ws/devel/setup.bash; roslaunch turtlebot_bringup turtlebot_robot.launch"
CMD bash -c "./wait-for-ros-nodes.sh"
