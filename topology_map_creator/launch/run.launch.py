from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topology_map_creator',  
            executable='topology_map_node',  
            name='topology_mapping',
            output='screen'
        )
    ])
