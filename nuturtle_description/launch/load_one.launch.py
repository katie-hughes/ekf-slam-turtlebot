
"""
Launches rviz with the turtlebot3 urdf file.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_path
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='use_jsp', default_value='true',
                              choices=['true', 'false'],
                              description='Choose if joint_state_publisher is launched'),
        DeclareLaunchArgument(name='use_rviz', default_value='true',
                              choices=['true', 'false'],
                              description='Choose if rviz is launched'),
        DeclareLaunchArgument(name='color', default_value='purple',
                              choices=['purple', 'red', 'green', 'blue'],
                              description='Change the color of the turtlebot'),

        DeclareLaunchArgument(name='model',
                              default_value=str(get_package_share_path('nuturtle_description') /
                                                'urdf/turtlebot3_burger.urdf.xacro'),
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig',
                              default_value=str(get_package_share_path('nuturtle_description') /
                                                'config/basic_purple.rviz'),
                              description='Absolute path to rviz config file'),

        Node(package='joint_state_publisher',
             executable='joint_state_publisher',
             condition=LaunchConfigurationEquals('use_jsp', 'true')),

        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description':
                          ParameterValue(Command(['xacro ',
                                                  LaunchConfiguration('model'),
                                                  ' color:=',
                                                  LaunchConfiguration('color')]),
                                         value_type=str)}]),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', LaunchConfiguration('rvizconfig')],
             condition=LaunchConfigurationEquals('use_rviz', 'true'),
             on_exit = Shutdown())
    ])