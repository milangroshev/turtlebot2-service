#!/bin/bash

source /opt/ros/kinetic/setup.bash

ros_nodes=($(rosnode list))

#printf '%s\n' "${ros_nodes[@]}"

in_array () {
        local somearray=${1}[@]
        shift
        for SEARCH_VALUE in "$@"; do
            FOUND=false
            for ARRAY_VALUE in ${!somearray}; do
                if [[ $ARRAY_VALUE == $SEARCH_VALUE ]]; then
                        FOUND=true
                        break
                fi
            done
            if ! $FOUND; then
                return 1
            fi
         done
         return 0
}

in_array ros_nodes "/map_server"
out=$?

while [ $out -eq 1 ]
do
        echo "Required ROS nodes are unavailable - sleeping"
        source /opt/ros/kinetic/setup.bash
        ros_nodes=($(rosnode list))
        in_array ros_nodes "/map_server"
        out=$?
        sleep 1
done

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun stage_ros stageros /home/turtlebot/catkin_ws/src/turtlebot_simulator/turtlebot_stage/maps/stage/twoRobotsMaze.world _base_watchdog_timeout:=0.5 /robot_0/odom:=/robot_0/odom /robot_0/base_pose_ground_truth:=/robot_0/base_pose_ground_truth /robot_0/cmd_vel:=/robot_0/mobile_base/commands/velocity /robot_0/base_scan:=/robot_0/scan /robot_1/odom:=/robot_1/odom /robot_1/base_pose_ground_truth:=/robot_1/base_pose_ground_truth /robot_1/cmd_vel:=/robot_1/mobile_base/commands/velocity /robot_1/base_scan:=/robot_1/scan
