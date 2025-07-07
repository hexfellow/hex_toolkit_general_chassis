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
            "period": rospy.get_param('~circle_period', 1.0),
            "inverse_flag": rospy.get_param('~circle_inverse_flag', False),
        }

        ### publisher
        self.__target_pose_pub = rospy.Publisher(
            'target_pose',
            PoseStamped,
            queue_size=10,
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
        while not rospy.is_shutdown():
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
            self.__tar_msg.header.stamp = rospy.Time.now()
            self.__target_pose_pub.publish(self.__tar_msg)

            # loop end process
            curr_target_idx = (curr_target_idx + 1) % self.__target_num
            self.__rate.sleep()
