FROM ros:kinetic
MAINTAINER mgroshev@pa.uc3m.es

WORKDIR /root

RUN groupadd -g 1000 turtlebot
RUN useradd -d /home/turtlebot -s /bin/bash -m turtlebot -u 1000 -g 1000 && echo "turtlebot:turtlebot" | chpasswd && adduser turtlebot sudo

RUN apt-get update && \
    apt-get install -y lsof xauth \
      python-rosinstall python-rosinstall-generator \
      ros-kinetic-tf \
      ros-kinetic-image-transport \
      ros-kinetic-joint-state-publisher \
      ros-kinetic-stage && \
    apt-get install -y ros-kinetic-librealsense || true && \
    printf '#!/bin/bash\nexit 0\n' > /var/lib/dpkg/info/ros-kinetic-librealsense.postinst && \
    dpkg --configure ros-kinetic-librealsense && \
    apt-get install -y -f && \
    apt-get install -y ros-kinetic-turtlebot-bringup


COPY launch/stage_ros.launch /opt/ros/kinetic/share/turtlebot_bringup/launch/

USER turtlebot
WORKDIR /home/turtlebot


RUN mkdir -p catkin_ws/src && cd catkin_ws/src && \
    git clone https://github.com/turtlebot/turtlebot_simulator.git && \
    git clone https://github.com/ros-simulation/stage_ros.git && \
    cd stage_ros && git checkout 1.8.0

RUN cd catkin_ws && bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"

RUN mkdir -p /home/turtlebot/scripts
COPY scripts/* /home/turtlebot/scripts/
COPY worlds/* /home/turtlebot/catkin_ws/src/turtlebot_simulator/turtlebot_stage/maps/stage/

WORKDIR /home/turtlebot/scripts/

USER root
RUN chmod +755 ros-run.sh

RUN rm -rf /opt/ros/kinetic/share/kobuki_node/param/base.yaml
COPY param/* /opt/ros/kinetic/share/kobuki_node/param/

USER turtlebot
ENV HOME /home/turtlebot
#CMD bash -c "source /opt/ros/kinetic/setup.bash; source catkin_ws/devel/setup.bash; roslaunch turtlebot_bringup turtlebot_robot.launch"
CMD bash -c "./ros-run.sh"
