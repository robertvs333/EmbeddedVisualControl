from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('jetson_camera'),
        'config',
        'parameters.yaml'
    )

    return LaunchDescription([
        Node(
            package='jetson_camera',
            executable='movement_node',
            name='movement_node',
            parameters=[config],
            output='screen'
        ),
        # Node(
        #     package='jetson_camera',
        #     executable='algorithm',
        #     name='algorithm_node',
        #     parameters=[config],
        #     output='screen'
        # ),
        # Node(
        #     package='jetson_camera',
        #     executable='publisher_node',
        #     name='camera_publisher',
        #     parameters=[config]
        # )
    
    ])

