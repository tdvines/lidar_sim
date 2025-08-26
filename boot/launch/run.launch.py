#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    boot_share = get_package_share_directory('boot')

    # Path to URDF and RViz config
    urdf_file = os.path.join(boot_share, 'models', 'ADcar_model', 'ADcar_model.urdf')
    rviz_file = os.path.join(boot_share, 'rviz_config', 'adcar.rviz')

    # Declare launch arguments
    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop',
        default_value='true',
        description='Enable teleop keyboard control'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include the simulation.launch.py
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(boot_share, 'launch', 'simulation.launch.py')
        )
    )

    # Function to conditionally add teleop node
    def add_teleop(context, *args, **kwargs):
        use_teleop = LaunchConfiguration('use_teleop').perform(context)
        nodes = []
        if use_teleop.lower() == 'true':
            nodes.append(
                Node(
                    package='teleop_twist_keyboard',
                    executable='teleop_twist_keyboard',
                    name='teleop_twist_keyboard',
                    output='screen',
                    prefix='xterm -hold -e',
                    arguments=[
                        '--ros-args', '-r', '__ns:=/ecart', '-r', 'cmd_vel:=/ecart/cmd_vel'
                    ]
                )
            )
        return nodes

    return LaunchDescription([
        use_teleop_arg,
        use_sim_time_arg,

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro ', urdf_file])
            }]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file]
        ),

        # PCL merge node
        Node(
            package='pcl_merge',
            executable='pcl_merge_node',
            name='pcl_merge_node',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'output_frame': 'chassis'},
                {'input_topics': [
                    "/scan_pointcloud",
                    "/scan_pointcloud_far_left", 
                    "/scan_pointcloud_far_right"
                ]},
                {'output_topic': 'scan_merged'},
            ],
            arguments=['--qos-reliability', 'reliable', '--qos-durability', 'transient_local']
        ),

        # Include simulation launch
        simulation_launch,

        # Optional teleop
        OpaqueFunction(function=add_teleop)
    ])
