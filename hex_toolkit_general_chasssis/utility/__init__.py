#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

import os

ROS_VERSION = os.environ.get('ROS_VERSION')
if ROS_VERSION == '1':
    from .ros1_cart_gen import CartGen as CartGen
    from .ros1_joy_ctrl import JoyCtrl as JoyCtrl
    from .ros1_key_ctrl import KeyCtrl as KeyCtrl
elif ROS_VERSION == '2':
    from .ros2_cart_gen import CartGen as CartGen
    from .ros2_joy_ctrl import JoyCtrl as JoyCtrl
    from .ros2_key_ctrl import KeyCtrl as KeyCtrl
else:
    raise ValueError("ROS_VERSION is not set")

__all__ = [
    "CartGen",
    "JoyCtrl",
    "KeyCtrl",
]
