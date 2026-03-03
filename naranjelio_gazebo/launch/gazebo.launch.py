from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('naranjelio_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'naranjelio.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf, 'use_sim_time': True}
        ]
    )

    # Launch modern Gazebo (Harmonic)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': 'empty.sdf'}.items()
    )

    # Bridge the /clock topic from Gazebo to ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Spawn the robot
    urdf_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'naranjelio',
            '-topic', 'robot_description',
            '-z', '0.5' # Drop it slightly above the ground
        ],
        output='screen'
    )

    # Spawn the Joint State Broadcaster
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawn the Joint Trajectory Controller
    load_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        clock_bridge,
        urdf_spawn_node,
        # We use TimerAction to delay the controllers slightly so Gazebo has time to load the robot first
        TimerAction(period=3.0, actions=[load_joint_state_broadcaster]),
        TimerAction(period=5.0, actions=[load_joint_trajectory_controller])
    ])