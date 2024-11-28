#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class DataCollector:
    def __init__(self):
        rospy.init_node('data_collector', anonymous=True)
        self.data = {
            "sensor_data": {},
            "context_features": {},
            "policy_data": {},
            "robot_state": {},
            "context_label": "Room with Static Objects"
        }
        self.rate = rospy.Rate(1)  # 10 Hz

        rospy.Subscriber('/robot_0/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/robot_0/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/robot_0/mobile_base/commands/velocity', Twist, self.velocity_callback)

    def lidar_callback(self, msg):
        self.data["sensor_data"]["lidar_ranges"] = list(msg.ranges)

    def odom_callback(self, msg):
        self.data["sensor_data"]["odometry"] = {
            "position": [msg.pose.pose.position.x, msg.pose.pose.position.y],
            "orientation": [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ],
        }

    def velocity_callback(self, msg):
        self.data["sensor_data"]["velocity"] = {
            "linear": msg.linear.x,
            "angular": msg.angular.z
        }

    def collect_data(self):
        while not rospy.is_shutdown():
            # Cargar los datos existentes
            try:
                with open('/home/ros/data_collector/dataset_static.json', 'r') as f:
                    dataset = json.load(f)
                if not isinstance(dataset, list):
                    rospy.logwarn("El archivo JSON no es una lista. Reemplazando con una lista vacía.")
                    dataset = []
            except IOError:
                dataset = []  # Inicializa como lista vacía si el archivo no existe

            # Añadir los nuevos datos al dataset
            dataset.append(self.data)

            # Guardar los datos actualizados
            with open('/home/ros/data_collector/dataset_static.json', 'w') as f:
                json.dump(dataset, f, indent=4)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        collector = DataCollector()
        collector.collect_data()
    except rospy.ROSInterruptException:
        pass
