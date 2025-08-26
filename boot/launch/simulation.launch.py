from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Path to your world file
    world_path = os.path.join(
        os.getenv('PWD', ''),  # current working directory
        'src', 'boot', 'worlds', 'simple_city.world'
    )

    # Path to your ADcar_model
    model_path = os.path.join(
        os.getenv('PWD', ''),
        'src', 'boot', 'models'
    )

    # Set GAZEBO_MODEL_PATH in the environment for this process
    env = os.environ.copy()
    # Prepend your model path so Gazebo finds it first
    if 'GAZEBO_MODEL_PATH' in env:
        env['GAZEBO_MODEL_PATH'] = model_path + ':' + env['GAZEBO_MODEL_PATH']
    else:
        env['GAZEBO_MODEL_PATH'] = model_path

    return LaunchDescription([
        # Launch Gazebo with the world file
        ExecuteProcess(
            cmd=['gazebo', world_path, '--verbose'],
            output='screen',
            env=env  # <-- pass the modified environment
        ),

        # Example ROS Node
        # Node(
        #     package='boot',
        #     executable='talker',
        #     name='boot_node',
        #     output='screen'
        # )
    ])
