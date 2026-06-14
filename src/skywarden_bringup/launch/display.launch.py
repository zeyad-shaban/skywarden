from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory("skywarden_description"), "urdf", "skywarden.xacro.urdf")
    rviz_config_path = os.path.join(get_package_share_directory("skywarden_bringup"), "config", "rviz.rviz")
    robot_description = Command([FindExecutable(name="xacro"), " ", urdf_path])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        condition=None if True else None,
    )

    print("====")
    print(rviz_config_path)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    ld = LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )

    return ld
