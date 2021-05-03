#!/bin/bash

source /opt/ros/kinetic/setup.bash


ROBOT=$ROBOT_NS

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

in_array ros_nodes "/map_server" "/stageros" "/$ROBOT/cmd_vel_mux"
out=$?

while [ $out -eq 1 ]
do
        echo "Required ROS nodes are unavailable - sleeping"
        source /opt/ros/kinetic/setup.bash
        ros_nodes=($(rosnode list))
        in_array ros_nodes "/map_server" "/stageros" "/$ROBOT/cmd_vel_mux"
        out=$?
        sleep 1
done

source /opt/ros/kinetic/setup.bash
roslaunch amcl amcl.launch --screen
