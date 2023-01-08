
"""
Launches rviz with the turtlebot3 urdf file.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_path
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    pkg_share = get_package_share_path('nuturtle_description')
    urdf_path = pkg_share / 'turtlebot3_burger.urdf'
    rviz_config_path = pkg_share / 'turtle_urdf.rviz'

    jsp_arg = DeclareLaunchArgument(name='use_jsp', default_value='true',
                                    choices=['true', 'false'],
                                    description='Choose if joint_state_publisher is launched')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(urdf_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig',
                                     default_value=str(rviz_config_path),
                                     description='Absolute path to rviz config file')
    robot_description = ParameterValue(LaunchConfiguration('model'), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Launch JSP if use_jsp is true.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=LaunchConfigurationEquals('use_jsp', 'true')
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        jsp_arg,
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ])