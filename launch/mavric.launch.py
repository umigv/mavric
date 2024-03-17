import os
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

   bno055_params = os.path.join(
        get_package_share_directory('bno055'),
        'config',
        'bno055_params.yaml'
        )
   bno055_driver_node = launch_ros.actions.Node(
        package = 'bno055',
        executable = 'bno055',
        parameters = [bno055_params]
    )
   bno055 = LaunchDescription([bno055_driver_node,
                                     RegisterEventHandler(
                                         event_handler=OnProcessExit(
                                             target_action=bno055_driver_node,
                                             on_exit=[EmitEvent(
                                                 event=Shutdown())],
                                         )),
                                     ])
   
   
   velodyne_params = os.path.join(
        get_package_share_directory('velodyne_driver'), 'config',
        'VLP16-velodyne_driver_node-params.yaml')
   
   velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[velodyne_params])
   
   velodyne = LaunchDescription([velodyne_driver_node,
                                     RegisterEventHandler(
                                         event_handler=OnProcessExit(
                                             target_action=velodyne_driver_node,
                                             on_exit=[EmitEvent(
                                                 event=Shutdown())],
                                         )),
                                     ])
   
   robot_localization = LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')],
           ),
        ])
   
   slam_toolbox = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         'broadcaster_listener_launch.py']),
      launch_arguments={'target_frame': 'carrot1'}.items(),
      )


   

   return LaunchDescription([
      bno055,
      velodyne,
      robot_localization,
      slam_toolbox
   ])
