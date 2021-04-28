FROM ros:kinetic
MAINTAINER mgroshev@pa.uc3m.es

WORKDIR /root
RUN useradd -m turtlebot && echo "turtlebot:turtlebot" | chpasswd && adduser turtlebot sudo

RUN \
  apt-get update && \
  apt-get install -y lsof && \
  apt-get install -y python-rosinstall python-rosinstall-generator && \
  apt-get install -y ros-kinetic-turtlebot-bringup


COPY launch/kobuki_rplidar_robot.launch /opt/ros/kinetic/share/turtlebot_bringup/launch/

USER turtlebot
WORKDIR /home/turtlebot


RUN mkdir -p catkin_ws/src && cd catkin_ws/src && \
    git clone https://github.com/Slamtec/rplidar_ros.git


COPY launch/kobuki_rplidar.launch catkin_ws/src/rplidar_ros/launch/

RUN cd catkin_ws && bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"


RUN mkdir -p /home/turtlebot/scripts
COPY scripts/* /home/turtlebot/scripts/
WORKDIR /home/turtlebot/scripts/

USER root
RUN chmod +755 wait-for-ros-nodes.sh

COPY rules/* /etc/udev/rules.d/
RUN chmod +755 /etc/udev/rules.d/rplidar.rules
RUN chmod +755 /etc/udev/rules.d/57-kobuki.rules

USER turtlebot
CMD bash -c "./wait-for-ros-nodes.sh"
