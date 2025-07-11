#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import hex_utils


class CircleGen:

    def __init__(self, name: str = "unknown"):
        ### ros node
        rospy.init_node(name, anonymous=True)
        self.__rate_param = {
            "ros": rospy.get_param('~rate_ros', 100.0),
        }
        self.__rate = rospy.Rate(self.__rate_param["ros"])

        ### pamameter
        # model
        self.__model_param = {
            "base": rospy.get_param('~model_base', "base_link"),
            "odom": rospy.get_param('~model_odom', "odom"),
        }
        # circle
        self.__circle_param = {
            "center": np.array(rospy.get_param('~circle_center', [0.0])),
            "radius": rospy.get_param('~circle_radius', 1.0),
            "arc_length": rospy.get_param('~arc_length', 0.1),
            "inverse_flag": rospy.get_param('~circle_inverse_flag', False),
        }

        ### publisher
        self.__target_pose_pub = rospy.Publisher(
            'target_pose',
            PoseStamped,
            queue_size=10,
        )

        ### subscriber
        self.__chassis_odom_sub = rospy.Subscriber(
            'odom', 
            Odometry, 
            self.__chassis_odom_callback
        )

        ### variable
        # current pose
        self.__current_pose = np.zeros(3)
        # arc_length to angle
        self.__arc_angle = self.__circle_param["arc_length"] / self.__circle_param["radius"]
        # target message
        self.__tar_msg = PoseStamped()
        self.__tar_msg.header.frame_id = self.__model_param["odom"]

    def __pose2d23d(
        self,
        x: float,
        y: float,
        yaw: float,
    ):
        pos = np.array([x, y, 0.0])
        quat = np.array([np.cos(yaw * 0.5), 0.0, 0.0, np.sin(yaw * 0.5)])
        return pos, quat

    def __chassis_odom_callback(self, msg: Odometry):
        self.__current_pose[0] = msg.pose.pose.position.x
        self.__current_pose[1] = msg.pose.pose.position.y
        qw = msg.pose.pose.orientation.w
        qz = msg.pose.pose.orientation.z
        self.__current_pose[2]= 2 * np.arctan2(qz, qw)    
    
    def __calculate_target_position(self):
        robot_x_rel = self.__current_pose[0] - self.__circle_param["center"][0]
        robot_y_rel = self.__current_pose[1] - self.__circle_param["center"][1]
        robot_angle = np.arctan2(robot_y_rel, robot_x_rel)
        target_angle = robot_angle + self.__arc_angle
        
        target_x = self.__circle_param["center"][0] + self.__circle_param["radius"] * np.cos(target_angle)
        target_y = self.__circle_param["center"][1] + self.__circle_param["radius"] * np.sin(target_angle)
        target_yaw = target_angle + np.pi/2
        
        return target_x, target_y, target_yaw

    def work(self):
        while not rospy.is_shutdown():
            # update target message
            target_x, target_y, target_yaw = self.__calculate_target_position()
            pos, quat = self.__pose2d23d(target_x, target_y, target_yaw)
            self.__tar_msg.pose.position.x = pos[0]
            self.__tar_msg.pose.position.y = pos[1]
            self.__tar_msg.pose.position.z = pos[2]
            self.__tar_msg.pose.orientation.w = quat[0]
            self.__tar_msg.pose.orientation.x = quat[1]
            self.__tar_msg.pose.orientation.y = quat[2]
            self.__tar_msg.pose.orientation.z = quat[3]

            # publish target message
            self.__tar_msg.header.stamp = rospy.Time.now()
            self.__target_pose_pub.publish(self.__tar_msg)

            self.__rate.sleep()
