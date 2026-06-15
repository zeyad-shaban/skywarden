from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    rviz = LaunchConfiguration("rviz", default="false")

    world = LaunchConfiguration("world", default=PathJoinSubstitution([FindPackageShare("skywarden_bringup"), "worlds", "my_world.sdf"]))

    robot_name = "skywarden"

    xacro_file = PathJoinSubstitution([FindPackageShare("skywarden_description"), "urdf", "skywarden.urdf.xacro"])

    controllers_file = PathJoinSubstitution([FindPackageShare("skywarden_bringup"), "config", "skywarden_controller.yaml"])

    robot_description_content = Command([FindExecutable(name="xacro"), " ", xacro_file])

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": use_sim_time},
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={
            "gz_args": [
                "-r -v 1 ",
                world,
            ]
        }.items(),
    )

    # fmt: off
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', '0',
            '-y', '0',
            '-z', '0',
            '-R', '0',
            '-P', '0',
            '-Y', '0',
        ],
    )
    # fmt: on

    bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
        ],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--param-file",
            controllers_file,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(rviz),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulated time",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz2",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("skywarden_bringup"),
                        "worlds",
                        "my_world.sdf",
                    ]
                ),
                description="Full path to the Gazebo world file",
            ),
            DeclareLaunchArgument("robot_name", default_value="skywarden"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.0"),
            DeclareLaunchArgument("roll", default_value="0.0"),
            DeclareLaunchArgument("pitch", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            gz_sim,
            bridge_clock,
            node_robot_state_publisher,
            gz_spawn_entity,
            rviz_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[joint_trajectory_controller_spawner],
                )
            ),
        ]
    )
