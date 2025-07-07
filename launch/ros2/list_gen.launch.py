#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-07-07
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
    # list target generator
    list_gen_path = FindPackageShare('hex_toolkit_general_chasssis').find(
        'hex_toolkit_general_chasssis') + '/config/ros2/list_gen.yaml'
    list_gen_node = Node(
        package='hex_toolkit_general_chasssis',
        executable='list_gen',
        name='list_gen',
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': False,
            },
            list_gen_path,
        ],
        remappings=[
            # publish
            ('/target_pose', '/target_pose'),
        ],
    )

    return LaunchDescription([
        # list target generator
        list_gen_node,
    ])
