#!/usr/bin/env python
import rospy
import roslib
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import paho.mqtt.client as paho
import json
import csv


amcl = open("/home/netcom/start/ros_logger/amcl.csv","w")
amcl_writer = csv.writer(amcl)
amcl_writer.writerow(["seq", "secs", "nsecs", "position_x", "position_y", "position_z", "orientation_x", "orientation_y", "orientation_z"])

odom = open("/home/netcom/start/ros_logger/odom.csv","w")
odom_writer = csv.writer(odom)
odom_writer.writerow(["seq", "secs", "nsecs", "position_x", "position_y", "position_z", "orientation_x", "orientation_y", "orientation_z", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"])

states = open("/home/netcom/start/ros_logger/joint_states.csv","w")
states_writer = csv.writer(states)
states_writer.writerow(["seq", "secs", "nsecs", "wheel_left_joint_position","wheel_right_joint_position", "wheel_left_joint_velocity","wheel_right_joint_velocity", "wheel_left_joint_effort","wheel_right_joint_effort"])

def robot1_amcl_pose_callback(msg):
    global amcl_writer
    seq=msg.header.seq
    secs=msg.header.stamp.secs
    nsecs=msg.header.stamp.nsecs
    pos_x=msg.pose.pose.position.x
    pos_y=msg.pose.pose.position.y
    pos_z=msg.pose.pose.position.z

    ori_x=msg.pose.pose.orientation.x
    ori_y=msg.pose.pose.orientation.y
    ori_z=msg.pose.pose.orientation.z
    
    amcl_writer.writerow([seq,secs,nsecs,pos_x,pos_y,pos_z,ori_x,ori_y,ori_z])

def robot1_odom_callback(msg):
    seq=msg.header.seq
    secs=msg.header.stamp.secs
    nsecs=msg.header.stamp.nsecs
    
    pos_x=msg.pose.pose.position.x
    pos_y=msg.pose.pose.position.y
    pos_z=msg.pose.pose.position.z

    ori_x=msg.pose.pose.orientation.x
    ori_y=msg.pose.pose.orientation.y
    ori_z=msg.pose.pose.orientation.z

    linear_x=msg.twist.twist.linear.x
    linear_y=msg.twist.twist.linear.y
    linear_z=msg.twist.twist.linear.z

    angular_x=msg.twist.twist.angular.x
    angular_y=msg.twist.twist.angular.y
    angular_z=msg.twist.twist.angular.z    

    odom_writer.writerow([seq,secs,nsecs,pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,linear_x,linear_y,linear_z,angular_x,angular_y,angular_z])
def robot1_joint_states_callback(msg):
    seq=msg.header.seq
    secs=msg.header.stamp.secs
    nsecs=msg.header.stamp.nsecs

    left_pos=msg.position[0]
    right_pos=msg.position[1]

    left_vel=msg.velocity[0]
    right_vel=msg.velocity[1]

    left_eff=msg.effort[0]
    right_eff=msg.effort[1]

    states_writer.writerow([seq,secs,nsecs,left_pos,right_pos,left_vel,right_vel,left_eff,right_eff])

if __name__ == '__main__':
    rospy.init_node('loc_ser', anonymous=False)
    rospy.Subscriber('robot1/amcl_pose', PoseWithCovarianceStamped, robot1_amcl_pose_callback)
    rospy.Subscriber('robot1/odom', Odometry, robot1_odom_callback)
    rospy.Subscriber('robot1/joint_states', JointState, robot1_joint_states_callback)
    rospy.spin()
