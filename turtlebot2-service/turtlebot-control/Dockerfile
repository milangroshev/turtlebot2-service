FROM ros:kinetic
MAINTAINER mgroshev@pa.uc3m.es

WORKDIR /root

RUN groupadd -g 1000 turtlebot
RUN useradd -d /home/turtlebot -s /bin/bash -m turtlebot -u 1000 -g 1000 && echo "turtlebot:turtlebot" | chpasswd && adduser turtlebot sudo

RUN apt-get update && \
    apt-get install -y lsof xauth \
      python-rosinstall python-rosinstall-generator \
      ros-kinetic-rviz \
      vim && \
    apt-get install -y ros-kinetic-librealsense || true && \
    printf '#!/bin/bash\nexit 0\n' > /var/lib/dpkg/info/ros-kinetic-librealsense.postinst && \
    dpkg --configure ros-kinetic-librealsense && \
    apt-get install -y -f && \
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
RUN chmod +755 map_navigation.py
USER turtlebot
ENV HOME /home/turtlebot
CMD ["python", "./map_navigation.py"]
#CMD bash -c "./wait-for-ros-nodes.sh"
