"""
Starts a SAS robot driver for a robot in a MuJoCo scene, simulated in the driver.

    ros2 launch sas_robot_driver_mujoco robot_launch.py model_path:=<scene.xml>

Arguments:
    model_path   The MJCF file of the scene (required).
    name         The name of the node, which is also the prefix of its topics (default: ur3e).
    config_file  The parameters of the robot (default: config/<name>.yaml of this package, e.g.,
                 config/ur3e.yaml or config/z1.yaml).
    headless     true to run without a window (default: false).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    name = LaunchConfiguration('name')
    config_file = LaunchConfiguration('config_file')
    model_path = LaunchConfiguration('model_path')
    headless = LaunchConfiguration('headless')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sigterm_timeout',
            default_value='30'
        ),
        DeclareLaunchArgument(
            'model_path',
            description='The MJCF file of the scene'
        ),
        DeclareLaunchArgument(
            'name',
            default_value='ur3e'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([FindPackageShare('sas_robot_driver_mujoco'), 'config', [name, '.yaml']])
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false'
        ),
        Node(
            output='screen',
            emulate_tty=True,
            package='sas_robot_driver_mujoco',
            executable='sas_robot_driver_mujoco_node',
            name=name,
            parameters=[config_file,
                        {'model_path': model_path,
                         'headless': ParameterValue(headless, value_type=bool)}]
        ),
    ])
