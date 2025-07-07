#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

import numpy as np

import rclpy
import rclpy.node
import threading
from geometry_msgs.msg import PoseStamped

import hex_utils


class CircleGen:

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
        self.__node.declare_parameter('circle_center', [0.0])
        self.__node.declare_parameter('circle_radius', 1.0)
        self.__node.declare_parameter('circle_period', 1.0)
        self.__node.declare_parameter('circle_inverse_flag', False)
        # model
        self.__model_param = {
            "base": self.__node.get_parameter('model_base').value,
            "odom": self.__node.get_parameter('model_odom').value,
        }
        # circle
        self.__circle_param = {
            "center":
            np.array(self.__node.get_parameter('circle_center').value),
            "radius": self.__node.get_parameter('circle_radius').value,
            "period": self.__node.get_parameter('circle_period').value,
            "inverse_flag":
            self.__node.get_parameter('circle_inverse_flag').value,
        }

        ### publisher
        self.__target_pose_pub = self.__node.create_publisher(
            PoseStamped,
            'target_pose',
            10,
        )

        ### variable
        # target list
        self.__target_num = int(self.__circle_param["period"] *
                                self.__rate_param["ros"])
        delta_theta = np.linspace(
            0,
            2 * np.pi,
            self.__target_num,
            endpoint=False,
        )
        target_pos = self.__circle_param[
            "center"] + self.__circle_param["radius"] * np.stack(
                [np.sin(delta_theta), np.cos(delta_theta)], axis=1)
        target_yaw = hex_utils.angle_norm(-delta_theta)
        self.__target_list = np.stack(
            [target_pos[:, 0], target_pos[:, 1], target_yaw], axis=1)
        if self.__circle_param["inverse_flag"]:
            self.__target_list = self.__target_list[::-1]
            self.__target_list[-1] *= -1
        # target message
        self.__tar_msg = PoseStamped()
        self.__tar_msg.header.frame_id = self.__model_param["odom"]

        ### spin thread
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

    def __spin(self):
        rclpy.spin(self.__node)

    def __pose2d23d(
        self,
        x: float,
        y: float,
        yaw: float,
    ):
        pos = np.array([x, y, 0.0])
        quat = np.array([np.cos(yaw * 0.5), 0.0, 0.0, np.sin(yaw * 0.5)])
        return pos, quat

    def work(self):
        curr_target_idx = 0
        while rclpy.ok():
            # update target message
            x, y, yaw = self.__target_list[curr_target_idx]
            pos, quat = self.__pose2d23d(x, y, yaw)
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

            # loop end process
            curr_target_idx = (curr_target_idx + 1) % self.__target_num
            self.__rate.sleep()
