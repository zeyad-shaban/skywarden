from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
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
    rviz_config_path = PathJoinSubstitution([FindPackageShare("skywarden_bringup"), "config", "rviz.rviz"])

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
                "-s -r -v 1 ",
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
            "/parrot_bebop/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
    )
    
    ros_gz_image_node = Node(
            package="ros_gz_image",
            executable="image_bridge",
            arguments=["/camera/image"],
            parameters=[{
                "camera.image.compressed.jpeg_quality": 75,
            }],
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
        arguments=['-d', rviz_config_path]
    )

    # Spawn Drone
    models_path = PathJoinSubstitution([FindPackageShare("threat_drone_sim"), "models"])
    drone_sdf_path = PathJoinSubstitution([models_path, "parrot_bebop", "model.sdf"])

    # fmt: off
    gz_spawn_drone_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'parrot_bebop',  # Must match the entity name used in your python script
                '-file', drone_sdf_path,
                '-x', '4.18',
                '-y', '-0.01',
                '-z', '1.64',
                '-R', '-0.01',
                '-P', '0.46',
                '-Y', '-3.06',
            ],
            output='screen'
        )

    # fmt: on
    
    controller_node = Node(
        package="skywarden_controller",
        executable="controller_node",
        output="screen",
        parameters=[{
            "Kyaw": 1.0,
            "Kpitch": 1.0,
        }],
        arguments=['-d', rviz_config_path]
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
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", models_path),
            gz_sim,
            bridge_clock,
            ros_gz_image_node,
            node_robot_state_publisher,
            gz_spawn_entity,
            gz_spawn_drone_node,
            rviz_node,
            controller_node,
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
