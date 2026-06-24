from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    rviz = LaunchConfiguration("rviz", default="false")
    visualize_ai = LaunchConfiguration("visualize_ai", default="false")

    rviz_config_path = PathJoinSubstitution([FindPackageShare("skywarden_bringup"), "config", "rviz.rviz"])
    world_path = PathJoinSubstitution([FindPackageShare("skywarden_bringup"), "worlds", "my_world.sdf"])
    xacro_file = PathJoinSubstitution([FindPackageShare("skywarden_description"), "urdf", "skywarden.urdf.xacro"])
    controllers_file = PathJoinSubstitution([FindPackageShare("skywarden_bringup"), "config", "skywarden_controller.yaml"])

    threat_drone_pkg_share = get_package_share_directory("threat_drone_sim")
    threat_drone_workspace_share = os.path.dirname(threat_drone_pkg_share)

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
                world_path,
            ]
        }.items(),
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', "skywarden",
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '0',
            '-R', '0', '-P', '0', '-Y', '0',
        ],
    )

    bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
    )
    
    # This specifically bridges the Pose Vector from Gazebo to the ROS 2 /tf topic
    gz_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_pose_bridge",
        arguments=[
            "/model/drone/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
        ],
        remappings=[
            ("/model/drone/pose", "/tf")
        ],
        output="screen",
    )

    ros_gz_image_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image"],
        parameters=[{"camera.image.compressed.jpeg_quality": 75}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster"],
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
        arguments=["-d", rviz_config_path]
    )

    # #### Spawn Drone
    models_path = PathJoinSubstitution([FindPackageShare("threat_drone_sim"), "models"])
    drone_sdf_path = PathJoinSubstitution([models_path, "drone", "model.sdf"])
    with open(os.path.join(threat_drone_pkg_share, "models", "drone", "model.sdf"), "r") as infp:
        drone_desc = infp.read()
        drone_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="drone_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": drone_desc,
                    "use_sim_time": use_sim_time,
                }
            ],
            remappings=[("/robot_description", "/drone_description")],
        )

    gz_spawn_drone_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'drone', 
                '-file', drone_sdf_path,
                '-x', '4.18', '-y', '-0.01', '-z', '1.64',
                '-R', '-0.01', '-P', '0.46', '-Y', '-3.06',
            ],
            output='screen'
        )

    controller_node = Node(
        package="skywarden_controller",
        executable="controller_node",
        output="screen",
        parameters=[{"Kyaw": 1.0, "Kpitch": 1.0}],
        arguments=["-d", rviz_config_path],
    )

    detector_node = Node(
        package="skywarden_ai",
        executable="detector_node.py",
        name="detector_node",
    )

    # visualize_ai_node = Node(
    #     package="skywarden_ai", 
    #     executable="visualizer_node.py", 
    #     name="visualizer_node", 
    #     condition=IfCondition(visualize_ai),
    # )
    
    visualize_ballistic_arc_node = Node(
        package="skywarden_ai", 
        executable="visualize_ballistic_arc.py", 
        name="visualize_ballistic_arc", 
        condition=IfCondition(rviz),
    )
    

    # TF Anchors to stitch the tree together
    turret_anchor_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='turret_world_anchor',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'footprint'],
        output='screen'
    )

    world_to_empty_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_empty_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'empty'],
        output='screen'
    )
    
    drone_to_body_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='drone_to_body_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'drone', 'body'],
        output='screen'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulated time"),
            DeclareLaunchArgument("rviz", default_value="false", description="Launch RViz2"),
            DeclareLaunchArgument("visualize_ai", default_value="false", description="Visualize AI Output"),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", threat_drone_workspace_share),
            
            gz_sim,
            bridge_clock,
            gz_pose_bridge,
            ros_gz_image_node,
            node_robot_state_publisher,
            gz_spawn_entity,
            gz_spawn_drone_node,
            drone_state_publisher_node,
            rviz_node,
            controller_node,
            # detector_node,
            # visualize_ai_node, # todo solve black screen issue here
            visualize_ballistic_arc_node,
            turret_anchor_node,
            world_to_empty_tf,
            drone_to_body_tf,
            
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