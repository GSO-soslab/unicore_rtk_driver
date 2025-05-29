from launch import LaunchDescription
import os
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_name = 'test'

    param_file = os.path.join(get_package_share_directory('unicore_rtk_driver'), 
                              'config', 'param.yaml') 


    return LaunchDescription([
        Node(
            package='unicore_rtk_driver',
            executable='unicore_rtk_driver_node.py',
            name='unicore_rtk_driver',
            namespace=robot_name,
            output='screen',
            parameters=[param_file]
        ),

        # Node(
        #     package='nmea_navsat_driver',
        #     executable='nmea_topic_driver',
        #     name='nmea_navsat_driver',
        #     namespace=robot_name,
        #     output='screen',
        #     remappings=[('nmea_sentence', 'unicore/nmea')]
        # ),
    ])
