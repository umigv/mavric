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
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("mavric_launch"),
                        "rviz",
                        "slam.rviz",
                    ),
                ],
            ),
        ]
    )
