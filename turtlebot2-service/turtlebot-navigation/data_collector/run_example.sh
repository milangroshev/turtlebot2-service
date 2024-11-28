docker run \
  --net host \
  --rm \
  -it \
  -v "$(pwd):/home/ros/data_collector" \
  ros:kinetic-ros-base \
  bash -c "source /opt/ros/kinetic/setup.bash && python /home/ros/data_collector/data_collector.py"
