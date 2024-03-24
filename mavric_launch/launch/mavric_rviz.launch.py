import os
import sys
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory
from yaml import safe_load

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    EmitEvent,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mavric_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("mavric_launch"),
                    "launch",
                    "mavric.launch.py",
                )
            ]
        )
    )

    rviz_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("mavric_launch"),
                    "launch",
                    "rviz.launch.py",
                )
            ]
        )
    )

    return LaunchDescription(
        [
            mavric_launch,
            rviz_launch,
        ]
    )
