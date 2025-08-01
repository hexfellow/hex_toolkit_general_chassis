#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-08-01
################################################################

import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)
from utility import OdomSim


def main():
    odom_sim = OdomSim("odom_sim")
    odom_sim.work()


if __name__ == '__main__':
    main()
