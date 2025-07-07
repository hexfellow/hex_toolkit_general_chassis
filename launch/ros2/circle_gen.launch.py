#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # circle target generator
    circle_gen_path = FindPackageShare('hex_toolkit_general_chasssis').find(
        'hex_toolkit_general_chasssis') + '/config/ros2/circle_gen.yaml'
    circle_gen_node = Node(
        package='hex_toolkit_general_chasssis',
        executable='circle_gen',
        name='circle_gen',
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': False,
            },
            circle_gen_path,
        ],
        remappings=[
            # publish
            ('/target_pose', '/target_pose'),
        ],
    )

    return LaunchDescription([
        # circle target generator
        circle_gen_node,
    ])
