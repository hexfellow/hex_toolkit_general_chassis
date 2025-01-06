#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-15
################################################################

import rclpy
import rclpy.node
import numpy as np
import threading
from threading import Lock
from pynput.keyboard import Listener, KeyCode, Key

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

KEY_REMAP = {
    "q": KeyCode.from_char('q'),
    "w": KeyCode.from_char('w'),
    "e": KeyCode.from_char('e'),
    "a": KeyCode.from_char('a'),
    "s": KeyCode.from_char('s'),
    "d": KeyCode.from_char('d'),
    "u": KeyCode.from_char('u'),
    "i": KeyCode.from_char('i'),
    "o": KeyCode.from_char('o'),
    "j": KeyCode.from_char('j'),
    "k": KeyCode.from_char('k'),
    "l": KeyCode.from_char('l'),
    "z": KeyCode.from_char('z'),
    "x": KeyCode.from_char('x'),
    "c": KeyCode.from_char('c'),
    "v": KeyCode.from_char('v'),
    "b": KeyCode.from_char('b'),
    "n": KeyCode.from_char('n'),
    "m": KeyCode.from_char('m'),
    "esc": Key.esc,
    "space": Key.space,
    "up": Key.up,
    "down": Key.down,
    "right": Key.right,
    "left": Key.left,
}


class KeyCtrl:

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
        self.__node.declare_parameter('ratio_vx', 0.0)
        self.__node.declare_parameter('ratio_vy', 0.0)
        self.__node.declare_parameter('ratio_yaw', 0.0)
        self.__node.declare_parameter('key_vx', ["unknown"])
        self.__node.declare_parameter('key_vy', ["unknown"])
        self.__node.declare_parameter('key_yaw', ["unknown"])
        # ratio
        self.__ratio_param = {
            "vx": self.__node.get_parameter('ratio_vx').value,
            "vy": self.__node.get_parameter('ratio_vy').value,
            "yaw": self.__node.get_parameter('ratio_yaw').value,
        }
        # key
        self.__key_param = {
            "vx": self.__node.get_parameter('key_vx').value,
            "vy": self.__node.get_parameter('key_vy').value,
            "yaw": self.__node.get_parameter('key_yaw').value,
        }

        ### publisher
        self.__unsafe_ctrl_pub = self.__node.create_publisher(
            Twist,
            'unsafe_ctrl',
            10,
        )
        self.__vel_ctrl_pub = self.__node.create_publisher(
            TwistStamped,
            'vel_ctrl',
            10,
        )

        ### variable
        # target vel
        self.__tar_vel = np.zeros(3)
        # vel message
        self.__unsafe_msg = Twist()
        self.__unsafe_msg.linear.x = 0.0
        self.__unsafe_msg.linear.y = 0.0
        self.__unsafe_msg.linear.z = 0.0
        self.__unsafe_msg.angular.x = 0.0
        self.__unsafe_msg.angular.y = 0.0
        self.__unsafe_msg.angular.z = 0.0
        self.__vel_msg = TwistStamped()
        self.__vel_msg.header.frame_id = "base_link"
        self.__vel_msg.twist.linear.x = 0.0
        self.__vel_msg.twist.linear.y = 0.0
        self.__vel_msg.twist.linear.z = 0.0
        self.__vel_msg.twist.angular.x = 0.0
        self.__vel_msg.twist.angular.y = 0.0
        self.__vel_msg.twist.angular.z = 0.0
        # keyboard
        self.__running = True
        self.__keyboard_lock = Lock()
        self.__keyboard_listener = Listener(
            on_press=self.__on_press,
            on_release=self.__on_release,
        )
        self.__keyboard_listener.start()

        ### spin thread
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

    def __spin(self):
        rclpy.spin(self.__node)

    def __del__(self):
        self.__keyboard_listener.stop()

    def __on_press(self, key):
        with self.__keyboard_lock:
            if key == KEY_REMAP[self.__key_param["vx"][0]]:
                self.__tar_vel[0] = -self.__ratio_param["vx"]
            elif key == KEY_REMAP[self.__key_param["vx"][1]]:
                self.__tar_vel[0] = self.__ratio_param["vx"]
            elif key == KEY_REMAP[self.__key_param["vy"][0]]:
                self.__tar_vel[1] = -self.__ratio_param["vy"]
            elif key == KEY_REMAP[self.__key_param["vy"][1]]:
                self.__tar_vel[1] = self.__ratio_param["vy"]
            elif key == KEY_REMAP[self.__key_param["yaw"][0]]:
                self.__tar_vel[2] = -self.__ratio_param["yaw"]
            elif key == KEY_REMAP[self.__key_param["yaw"][1]]:
                self.__tar_vel[2] = self.__ratio_param["yaw"]

    def __on_release(self, key):
        with self.__keyboard_lock:
            if key == Key.esc:
                self.__running = False
            elif key == KEY_REMAP[self.__key_param["vx"][
                    0]] or key == KEY_REMAP[self.__key_param["vx"][1]]:
                self.__tar_vel[0] = 0.0
            elif key == KEY_REMAP[self.__key_param["vy"][
                    0]] or key == KEY_REMAP[self.__key_param["vy"][1]]:
                self.__tar_vel[1] = 0.0
            elif key == KEY_REMAP[self.__key_param["yaw"][
                    0]] or key == KEY_REMAP[self.__key_param["yaw"][1]]:
                self.__tar_vel[2] = 0.0

    def work(self):
        while rclpy.ok() and self.__running:
            # update message
            self.__unsafe_msg.linear.x = self.__tar_vel[0]
            self.__unsafe_msg.linear.y = self.__tar_vel[1]
            self.__unsafe_msg.angular.z = self.__tar_vel[2]
            self.__vel_msg.twist.linear.x = self.__tar_vel[0]
            self.__vel_msg.twist.linear.y = self.__tar_vel[1]
            self.__vel_msg.twist.angular.z = self.__tar_vel[2]

            # publish message
            self.__unsafe_ctrl_pub.publish(self.__unsafe_msg)
            self.__vel_msg.header.stamp = self.__node.get_clock().now().to_msg(
            )
            self.__vel_ctrl_pub.publish(self.__vel_msg)

            # sleep
            self.__rate.sleep()
