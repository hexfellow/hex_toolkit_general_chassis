#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-25
################################################################

import rospy
import pygame
import numpy as np
import pygame.locals as pygconst

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

JOY_BUTTON_MAP = {
    "L1": 6,
    "L2": 8,
    "R1": 7,
    "R2": 9,
    "A": 0,
    "B": 1,
    "X": 3,
    "Y": 4,
}

JOY_AXIS_MAP = {
    "LX": 0,
    "LY": 1,
    "RX": 2,
    "RY": 3,
}

JOY_MOTION_MAP = {
    "UP": (0, 1),
    "DOWN": (0, -1),
    "LEFT": (-1, 0),
    "RIGHT": (1, 0),
}


class JoyCtrl:

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
        # joy
        self.__joy_param = {
            "deadzone": rospy.get_param('~joy_deadzone', 1.0),
            "vx": rospy.get_param('~joy_vx', "unknown"),
            "vy": rospy.get_param('~joy_vy', "unknown"),
            "yaw": rospy.get_param('~joy_yaw', "unknown"),
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
        # joystick
        self.__running = True
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() != 0:
            self.__joystick = pygame.joystick.Joystick(0)
            self.__joystick.init()
            print(f"Connected to joystick: {self.__joystick.get_name()}")
        else:
            rospy.logfatal(
                "No joystick detected. Please connect the joystick and restart the program."
            )
            exit(1)

    def __del__(self):
        pygame.joystick.quit()
        pygame.quit()

    def __update_vel(self):
        for event in pygame.event.get():
            if event.type == pygconst.QUIT:
                self.__running = False
            elif event.type == pygconst.JOYAXISMOTION:
                if event.axis == JOY_AXIS_MAP[self.__joy_param["vx"]]:
                    value = self.__deadzone(event.value,
                                            self.__joy_param["deadzone"])
                    self.__tar_vel[0] = self.__ratio_param["vx"] * value
                elif event.axis == JOY_AXIS_MAP[self.__joy_param["vy"]]:
                    value = self.__deadzone(event.value,
                                            self.__joy_param["deadzone"])
                    self.__tar_vel[1] = self.__ratio_param["vy"] * value
                elif event.axis == JOY_AXIS_MAP[self.__joy_param["yaw"]]:
                    value = self.__deadzone(event.value,
                                            self.__joy_param["deadzone"])
                    self.__tar_vel[2] = self.__ratio_param["yaw"] * value

    def __deadzone(self, value: float, deadzone: float) -> float:
        if np.fabs(value) < deadzone:
            return 0.0
        else:
            return value

    def work(self):
        while not rospy.is_shutdown() and self.__running:
            # joystick
            self.__update_vel()

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
