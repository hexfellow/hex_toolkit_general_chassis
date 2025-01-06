#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-24
################################################################

import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)
from utility import JoyCtrl


def main():
    joy_ctrl = JoyCtrl("joy_ctrl")
    joy_ctrl.work()


if __name__ == '__main__':
    main()
