#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-25
################################################################

import rospy
import numpy as np
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
        rospy.init_node(name, anonymous=True)
        self.__rate_param = {
            "ros": rospy.get_param('~rate_ros', 100.0),
        }
        self.__rate = rospy.Rate(self.__rate_param["ros"])

        ### pamameter
        # ratio
        self.__ratio_param = {
            "vx": rospy.get_param('~ratio_vx', 0.0),
            "vy": rospy.get_param('~ratio_vy', 0.0),
            "yaw": rospy.get_param('~ratio_yaw', 0.0),
        }
        # key
        self.__key_param = {
            "vx": rospy.get_param('~key_vx', ["unknown"]),
            "vy": rospy.get_param('~key_vy', ["unknown"]),
            "yaw": rospy.get_param('~key_yaw', ["unknown"]),
        }

        ### publisher
        self.__unsafe_ctrl_pub = rospy.Publisher(
            'unsafe_ctrl',
            Twist,
            queue_size=10,
        )
        self.__vel_ctrl_pub = rospy.Publisher(
            'vel_ctrl',
            TwistStamped,
            queue_size=10,
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
        while not rospy.is_shutdown() and self.__running:
            # update message
            self.__unsafe_msg.linear.x = self.__tar_vel[0]
            self.__unsafe_msg.linear.y = self.__tar_vel[1]
            self.__unsafe_msg.angular.z = self.__tar_vel[2]
            self.__vel_msg.twist.linear.x = self.__tar_vel[0]
            self.__vel_msg.twist.linear.y = self.__tar_vel[1]
            self.__vel_msg.twist.angular.z = self.__tar_vel[2]

            # publish message
            self.__unsafe_ctrl_pub.publish(self.__unsafe_msg)
            self.__vel_msg.header.stamp = rospy.Time.now()
            self.__vel_ctrl_pub.publish(self.__vel_msg)

            # sleep
            self.__rate.sleep()
