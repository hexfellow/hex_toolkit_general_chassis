#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-01-23
################################################################

import threading
import numpy as np

import rclpy
import rclpy.node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from hex_utils import ObsUtilWork
from hex_utils import HexCartState


class OdomSim:

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

        ### pamameter
        # declare parameters
        self.__node.declare_parameter('model_base', "base_link")
        self.__node.declare_parameter('model_odom', "odom")
        self.__node.declare_parameter('limit_vel', [0.0])
        self.__node.declare_parameter('limit_acc', [0.0])
        # model
        self.__model_param = {
            "base": self.__node.get_parameter('model_base').value,
            "odom": self.__node.get_parameter('model_odom').value,
        }
        # limit
        self.__limit_param = {
            "vel": np.array(self.__node.get_parameter('limit_vel').value),
            "acc": np.array(self.__node.get_parameter('limit_acc').value),
        }

        ### publisher
        self.__odom_pub = self.__node.create_publisher(
            Odometry,
            'odom',
            10,
        )

        ### subscriber
        self.__twist_sub = self.__node.create_subscription(
            Twist,
            'cmd_vel',
            self.__twist_callback,
            10,
        )

        ### tf
        self.__tf_broadcaster = TransformBroadcaster(self.__node)

        ### utility
        self.__dt = 1.0 / self.__rate_param["ros"]
        self.__obs_util = ObsUtilWork(
            dt=self.__dt,
            vel_limit=self.__limit_param["vel"],
            acc_limit=self.__limit_param["acc"],
        )
        self.__vel_lock = threading.Lock()
        self.__obs_util.set_state(HexCartState())

        ### variable
        self.__vel_tar = np.zeros(3)
        self.__omega_tar = np.zeros(3)
        self.__odom_msg = Odometry()
        self.__odom_msg.child_frame_id = self.__model_param["base"]
        self.__odom_msg.header.frame_id = self.__model_param["odom"]
        self.__odom_msg.pose.pose.position.x = 0.0
        self.__odom_msg.pose.pose.position.y = 0.0
        self.__odom_msg.pose.pose.position.z = 0.0
        self.__odom_msg.pose.pose.orientation.x = 0.0
        self.__odom_msg.pose.pose.orientation.y = 0.0
        self.__odom_msg.pose.pose.orientation.z = 0.0
        self.__odom_msg.pose.pose.orientation.w = 1.0
        self.__odom_msg.twist.twist.linear.x = 0.0
        self.__odom_msg.twist.twist.linear.y = 0.0
        self.__odom_msg.twist.twist.linear.z = 0.0
        self.__odom_msg.twist.twist.angular.x = 0.0
        self.__odom_msg.twist.twist.angular.y = 0.0
        self.__odom_msg.twist.twist.angular.z = 0.0
        self.__tf_msg = TransformStamped()
        self.__tf_msg.child_frame_id = self.__model_param["base"]
        self.__tf_msg.header.frame_id = self.__model_param["odom"]
        self.__tf_msg.transform.translation.x = 0.0
        self.__tf_msg.transform.translation.y = 0.0
        self.__tf_msg.transform.translation.z = 0.0
        self.__tf_msg.transform.rotation.x = 0.0
        self.__tf_msg.transform.rotation.y = 0.0
        self.__tf_msg.transform.rotation.z = 0.0
        self.__tf_msg.transform.rotation.w = 1.0

        ### spin thread
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

    def __spin(self):
        rclpy.spin(self.__node)

    def __twist_callback(self, msg: Twist):
        with self.__vel_lock:
            self.__vel_tar = np.array([
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
            ])
            self.__omega_tar = np.array([
                msg.angular.x,
                msg.angular.y,
                msg.angular.z,
            ])

    def work(self):
        while rclpy.ok():
            # get current state
            cur_state = self.__obs_util.get_state()
            vel_cur = cur_state.vel().get_linear()
            omega_cur = cur_state.vel().get_angular()

            # predict next state
            with self.__vel_lock:
                acc_tar = (self.__vel_tar - vel_cur) / self.__dt
                alpha_tar = (self.__omega_tar - omega_cur) / self.__dt
            self.__obs_util.predict(
                acc_lin=acc_tar,
                acc_ang=alpha_tar,
            )
            cur_state = self.__obs_util.get_state()

            # update messages
            odom_pos = cur_state.pose().get_pos()
            odom_quat = cur_state.pose().get_quat()
            odom_vel = cur_state.vel().get_linear()
            odom_omega = cur_state.vel().get_angular()
            self.__odom_msg.pose.pose.position.x = odom_pos[0]
            self.__odom_msg.pose.pose.position.y = odom_pos[1]
            self.__odom_msg.pose.pose.position.z = odom_pos[2]
            self.__odom_msg.pose.pose.orientation.w = odom_quat[0]
            self.__odom_msg.pose.pose.orientation.x = odom_quat[1]
            self.__odom_msg.pose.pose.orientation.y = odom_quat[2]
            self.__odom_msg.pose.pose.orientation.z = odom_quat[3]
            self.__odom_msg.twist.twist.linear.x = odom_vel[0]
            self.__odom_msg.twist.twist.linear.y = odom_vel[1]
            self.__odom_msg.twist.twist.linear.z = odom_vel[2]
            self.__odom_msg.twist.twist.angular.x = odom_omega[0]
            self.__odom_msg.twist.twist.angular.y = odom_omega[1]
            self.__odom_msg.twist.twist.angular.z = odom_omega[2]
            self.__tf_msg.transform.translation.x = odom_pos[0]
            self.__tf_msg.transform.translation.y = odom_pos[1]
            self.__tf_msg.transform.translation.z = odom_pos[2]
            self.__tf_msg.transform.rotation.w = odom_quat[0]
            self.__tf_msg.transform.rotation.x = odom_quat[1]
            self.__tf_msg.transform.rotation.y = odom_quat[2]
            self.__tf_msg.transform.rotation.z = odom_quat[3]

            # publish messages
            self.__odom_msg.header.stamp = self.__node.get_clock().now(
            ).to_msg()
            self.__tf_msg.header.stamp = self.__odom_msg.header.stamp
            self.__odom_pub.publish(self.__odom_msg)
            self.__tf_broadcaster.sendTransform(self.__tf_msg)

            # sleep
            self.__rate.sleep()
