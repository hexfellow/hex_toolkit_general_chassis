#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

import json
import numpy as np

import rclpy
import rclpy.node
import threading
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import hex_utils


class ListGen:

    def __init__(self, name: str = "unknown"):
        ### ros node
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()
        self.__node.declare_parameter('rate_ros', 100.0)
        self.__rate_param = {
            "ros": self.__node.get_parameter('rate_ros').value,
        }
        self.__rate = self.__node.create_rate(self.__rate_param["ros"])

        ### parameter
        # declare parameters
        self.__node.declare_parameter('model_base', "unknown")
        self.__node.declare_parameter('model_odom', "unknown")
        self.__node.declare_parameter('list_target', ["[0.0, 0.0, 0.0]"])
        self.__node.declare_parameter('switch_distance', 0.1)
        self.__node.declare_parameter('list_inverse_flag', False)
        # model
        self.__model_param = {
            "base": self.__node.get_parameter('model_base').value,
            "odom": self.__node.get_parameter('model_odom').value,
        }
        # list
        self.__list_param = {
            "target":
            np.array(
                self.__str_to_list(
                    self.__node.get_parameter('list_target').value)),
            "switch_distance":
            self.__node.get_parameter('switch_distance').value,
            "inverse_flag":
            self.__node.get_parameter('list_inverse_flag').value,
        }

        ### publisher
        self.__target_pose_pub = self.__node.create_publisher(
            PoseStamped,
            'target_pose',
            10,
        )

        ### subscriber
        self.__chassis_odom_sub = self.__node.create_subscription(
            Odometry,
            'odom', 
            self.__chassis_odom_callback,
            10,
        )

        ### variable
        # current position
        self.__current_pose = np.zeros(3)
        # target list
        self.__switch_distance = self.__list_param["switch_distance"] 
        self.__target_list = self.__list_param["target"]
        if self.__list_param["inverse_flag"]:
            self.__target_list = self.__target_list[::-1]
            self.__target_list[:, 2] *= -1
        # target message
        self.__tar_msg = PoseStamped()
        self.__tar_msg.header.frame_id = self.__model_param["odom"]

        ### spin thread
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

    def __spin(self):
        rclpy.spin(self.__node)

    def __str_to_list(self, list_str) -> list:
        result = []
        for s in list_str:
            l = json.loads(s)
            result.append(l)
        return result

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

    def work(self):
        curr_target_idx = 0
        while rclpy.ok():
            # update target message
            target = self.__target_list[curr_target_idx]
            dist = np.linalg.norm(self.__current_pose[:2]-target[:2])
            if dist < self.__switch_distance:
                curr_target_idx = (curr_target_idx + 1) % len(self.__target_list)
            pos, quat = self.__pose2d23d(target[0], target[1], target[2])
            self.__tar_msg.pose.position.x = pos[0]
            self.__tar_msg.pose.position.y = pos[1]
            self.__tar_msg.pose.position.z = pos[2]
            self.__tar_msg.pose.orientation.w = quat[0]
            self.__tar_msg.pose.orientation.x = quat[1]
            self.__tar_msg.pose.orientation.y = quat[2]
            self.__tar_msg.pose.orientation.z = quat[3]

            # publish target message
            self.__tar_msg.header.stamp = self.__node.get_clock().now().to_msg(
            )
            self.__target_pose_pub.publish(self.__tar_msg)

            self.__rate.sleep()
