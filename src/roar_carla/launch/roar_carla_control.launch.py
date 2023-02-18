# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = launch.LaunchDescription()

    node = Node(
        name="roar_carla_controller",
        executable="carla_controller_converter",
        package="roar_carla",
    )

    # node
    ld.add_action(node)

    return ld
