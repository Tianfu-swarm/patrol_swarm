from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Create LaunchDescription
    ld = LaunchDescription()

    # Launch Gazebo server with factory
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'gzserver_args': '--factory'}.items()  # Add factory argument
    )
    ld.add_action(gazebo_server)

    # Launch Gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    ld.add_action(gazebo_client)

    # Spawn 10 robots
    for i in range(10):  # Loop to spawn 10 robots
        namespace = f'robot_{i}'
        x_pose = str(-2.0 + i * 0.5)  # Adjust x pose
        y_pose = '-0.5'  # Keep y pose constant

        spawn_turtlebot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'entity': f'burger_{i}',  # Ensure each robot has a unique entity name
                'x_pose': x_pose,
                'y_pose': y_pose,
                'use_sim_time': use_sim_time
            }.items()
        )
        ld.add_action(spawn_turtlebot)



    return ld
