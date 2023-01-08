
"""
Launches rviz with my turtlebot urdf file.
Launchfile format taken from this ros tutorial:
https://github.com/ros/urdf_tutorial/blob/ros2/launch/display.launch.py
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
    urdf_path = pkg_share / 'turtle.urdf.xacro'
    rviz_config_path = pkg_share / 'turtle_urdf.rviz'
    rviz_config_path_base_fixed = pkg_share / 'turtle_urdf_base_fixed.rviz'

    jsp_arg = DeclareLaunchArgument(name='use_jsp', default_value='gui',
                                    choices=['gui', 'jsp', 'none'],
                                    description='Choose which joint state publisher to run')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(urdf_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig',
                                     default_value=str(rviz_config_path_base_fixed),
                                     choices=[str(rviz_config_path),
                                              str(rviz_config_path_base_fixed)],
                                     description='Absolute path to rviz config file')
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    # (or neither if it's none)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        # if the jsp argument is jsp
        # I copied the format from here:
        # https://answers.ros.org/question/360264/ifconditionpythonexpression-am-i-doing-it-right/
        condition=LaunchConfigurationEquals('use_jsp', 'jsp')
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        # if the jsp argument is gui
        # again format copied from here:
        # https://answers.ros.org/question/360264/ifconditionpythonexpression-am-i-doing-it-right/
        condition=LaunchConfigurationEquals('use_jsp', 'gui')
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
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])