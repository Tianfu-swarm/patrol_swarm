from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # World file path
    world_file = '/home/tianfu/patrol_ws/src/patrol_swarm/simulate/world/empty.world'

    # Define namespaces for each robot
    namespaces = ['robot1', 'robot2', 'robot3']

    # Create a list to hold the robot spawn commands
    spawn_robots = []

    for ns in namespaces:
        spawn_robots.append(
            GroupAction([
                ExecuteProcess(
                    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
                    output='screen'
                ),
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-entity', ns, '-file', os.path.join(FindPackageShare('turtlebot4_description').find('turtlebot4_description'), 'urdf', 'turtlebot4.urdf'), '-robot_namespace', ns],
                    output='screen'
                )
            ])
        )

    return LaunchDescription(spawn_robots)
