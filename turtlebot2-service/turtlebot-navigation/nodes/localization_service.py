#!/usr/bin/env python
import rospy
import roslib
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
import paho.mqtt.client as paho
import json

platform="172.1.0.8"

client= paho.Client("localization")

def robot1_amcl_pose_callback(msg):
    x1=msg.pose.pose.position.x
    y1=msg.pose.pose.position.y
    MQTT_MSG=json.dumps({"center": [x1,y1],"radius":  3});
    client.publish("/experiment/location",MQTT_MSG)
    #print ("(x1:{0},y1:{1})".format(x1,y1))


if __name__ == '__main__':
    rospy.init_node('loc_ser', anonymous=False)
    rospy.Subscriber('robot1/amcl_pose', PoseWithCovarianceStamped, robot1_amcl_pose_callback)
    client.connect(platform)
    client.loop_start()
    rospy.spin()
