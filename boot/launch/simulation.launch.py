from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #  Get boot package share dir
    boot_share = get_package_share_directory('boot')
    world_path = os.path.join(boot_share, 'worlds', 'simple_city.world')
    model_path = os.path.join(boot_share, 'models')

    # Relative path to citysim from boot package
    workspace_root = os.path.join(boot_share, '..', '..')  # ../../ from boot/share
    citysim_plugin_path = os.path.join(workspace_root, 'citysim', 'lib', 'citysim-0', 'plugins')
    print (citysim_plugin_path)


    # Environment
    env = os.environ.copy()
    env['GAZEBO_MODEL_PATH'] = model_path + ':' + env.get('GAZEBO_MODEL_PATH', '')
    env['GAZEBO_PLUGIN_PATH'] = citysim_plugin_path + ':' + env.get('GAZEBO_PLUGIN_PATH', '')


    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', world_path, '--verbose'],
            output='screen',
            env=env
        ),
        # Example Node (uncomment if needed)
        # Node(
        #     package='boot',
        #     executable='talker',
        #     name='boot_node',
        #     output='screen'
        # )
    ])





