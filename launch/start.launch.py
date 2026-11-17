import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('unicore_rtk_driver'),
                                  'config', 'param.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='test'),
        DeclareLaunchArgument('node_name', default_value='unicore_rtk_driver'),
        DeclareLaunchArgument('params_file', default_value=default_params),

        Node(
            package='unicore_rtk_driver',
            executable='unicore_rtk_driver_node',
            name=LaunchConfiguration('node_name'),
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
