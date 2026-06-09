FROM ros:kinetic
MAINTAINER mgroshev@pa.uc3m.es

WORKDIR /root
RUN useradd -ms /bin/bash turtlebot

RUN apt-get update && \
    apt-get install -y lsof \
      python-rosinstall python-rosinstall-generator \
      ros-kinetic-move-base && \
    apt-get install -y ros-kinetic-librealsense || true && \
    printf '#!/bin/bash\nexit 0\n' > /var/lib/dpkg/info/ros-kinetic-librealsense.postinst && \
    dpkg --configure ros-kinetic-librealsense && \
    apt-get install -y -f && \
    apt-get install -y ros-kinetic-turtlebot-apps


COPY launch/* /opt/ros/kinetic/share/turtlebot_navigation/launch/
COPY nodes/* /opt/ros/kinetic/share/turtlebot_navigation/nodes/
RUN rm -rf /opt/ros/kinetic/share/turtlebot_navigation/param/
COPY param/* /opt/ros/kinetic/share/turtlebot_navigation/param/

USER turtlebot
WORKDIR /home/turtlebot

RUN mkdir -p /home/turtlebot/scripts
COPY scripts/* /home/turtlebot/scripts/
WORKDIR /home/turtlebot/scripts/

USER root
RUN chmod +755 wait-for-ros-nodes.sh
USER turtlebot

CMD bash -c "./wait-for-ros-nodes.sh"