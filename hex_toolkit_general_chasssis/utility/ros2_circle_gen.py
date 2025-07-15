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
from nav_msgs.msg import Odometry

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
        self.__node.declare_parameter('circle_interpolation', 1_000)
        self.__node.declare_parameter('circle_switch_dist', 0.1)
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
            "interpolation":
            self.__node.get_parameter('circle_interpolation').value,
            "switch_dist":
            self.__node.get_parameter('circle_switch_dist').value,
            "inverse_flag":
            self.__node.get_parameter('circle_inverse_flag').value,
            "fixed_head": self.__node.get_parameter('circle_fixed_head').value,
        }

        ### publisher
        self.__target_pose_pub = self.__node.create_publisher(
            PoseStamped,
            'target_pose',
            10,
        )

        ### subscriber
        self.__odom_sub = self.__node.create_subscription(
            Odometry,
            'odom',
            self.__odom_callback,
            10,
        )

        ### variable
        # target list
        self.__target_list = []
        for i in range(self.__circle_param["interpolation"]):
            ratio = i / self.__circle_param["interpolation"]
            x = self.__circle_param["center"][
                0] + self.__circle_param["radius"] * np.cos(ratio * 2 * np.pi)
            y = self.__circle_param["center"][
                1] + self.__circle_param["radius"] * np.sin(ratio * 2 * np.pi)
            yaw = 0.0 if self.__circle_param[
                "fixed_head"] else hex_utils.angle_norm(
                    np.arctan2(y, x) + np.pi / 2.0)
            self.__target_list.append([x, y, yaw])
        self.__target_list = np.array(self.__target_list)
        if self.__circle_param["inverse_flag"]:
            self.__target_list = self.__target_list[::-1]
            if not self.__circle_param["fixed_head"]:
                self.__target_list[:, 2] = hex_utils.angle_norm(
                    self.__target_list[:, 2] + np.pi)
        self.__total_num = self.__target_list.shape[0]

        # curr
        self.__curr_pose = np.array([0.0, 0.0, 0.0])

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

    def __odom_callback(self, msg: Odometry):
        self.__curr_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            hex_utils.quat2yaw(
                np.array([
                    msg.pose.pose.orientation.w,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                ])),
        ])

    def work(self):
        curr_idx = 0
        while rclpy.ok():
            while np.linalg.norm(self.__curr_pose[:2] -
                                 self.__target_list[curr_idx][:2]
                                 ) < self.__circle_param["switch_dist"]:
                curr_idx = (curr_idx + 1) % self.__total_num

            # update curr pose
            self.__curr_pose = self.__target_list[curr_idx]

            # update target message
            x, y, yaw = self.__target_list[curr_idx]
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
            self.__rate.sleep()
