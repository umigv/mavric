import os
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory
from yaml import safe_load

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    bno055_params = os.path.join(
        get_package_share_directory("bno055"), "config", "bno055_params.yaml"
    )
    bno055_driver_node = launch_ros.actions.Node(
        package="bno055", executable="bno055", parameters=[bno055_params]
    )
    bno055 = LaunchDescription(
        [
            bno055_driver_node,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=bno055_driver_node,
                    on_exit=[EmitEvent(event=Shutdown())],
                )
            ),
        ]
    )
    driver_share_dir = get_package_share_directory('mavric_launch')
    driver_params_file = os.path.join(driver_share_dir, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params_file])

    convert_share_dir = get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(convert_share_dir, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(convert_share_dir, 'params', 'VLP16db.yaml')
    velodyne_transform_node = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                      executable='velodyne_transform_node',
                                                      output='both',
                                                      parameters=[convert_params])

    laserscan_share_dir = get_package_share_directory('velodyne_laserscan')
    laserscan_params_file = os.path.join(laserscan_share_dir, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node = launch_ros.actions.Node(package='velodyne_laserscan',
                                                      executable='velodyne_laserscan_node',
                                                      output='both',
                                                      parameters=[laserscan_params_file])


    velodyne = LaunchDescription([velodyne_driver_node,
                                     velodyne_transform_node,
                                     velodyne_laserscan_node,

                                     RegisterEventHandler(
                                         event_handler=OnProcessExit(
                                             target_action=velodyne_driver_node,
                                             on_exit=[EmitEvent(
                                                 event=Shutdown())],
                                         )),
                                     ])

    robot_localization = LaunchDescription(
        [
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("mavric_launch"),
                        "config",
                        "mavric_ekf.yaml",
                    )
                ],
            ),
        ]
    )

    slam_params_file = LaunchConfiguration('slam_params_file')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("mavric_launch"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    slam_toolbox = LaunchDescription()

    slam_toolbox.add_action(declare_use_sim_time_argument)
    slam_toolbox.add_action(declare_slam_params_file_cmd)
    slam_toolbox.add_action(start_async_slam_toolbox_node)
    urdf = os.path.join(
        get_package_share_directory('mavric_launch'),
        'urdf',
        "mavric.urdf")

    # Major refactor of the robot_state_publisher
    # Reference page: https://github.com/ros2/demos/pull/426
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    # print (robot_desc) # Printing urdf information.

    rsp = LaunchDescription([
        declare_use_sim_time_argument,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}]),
    ])

    return LaunchDescription([
        rsp,
        bno055,
        velodyne, 
        robot_localization,
        slam_toolbox
    ])
