import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 定义机器人的模型和数量
    turtlebot3_model_value = 'burger'
    robot_count_value = 10  # 设置要生成的机器人数量
    use_sim_time_value = 'true'  # 使用仿真时间

    # Create LaunchDescription
    ld = LaunchDescription()

    # Declare the launch arguments (optional, but good for clarity)
    declare_turtlebot3_model_cmd = DeclareLaunchArgument(
        'turtlebot3_model',
        default_value=turtlebot3_model_value,
        description='The model of the TurtleBot3 to spawn'
    )
    
    declare_robot_count_cmd = DeclareLaunchArgument(
        'robot_count',
        default_value=str(robot_count_value),  # 将整数转换为字符串
        description='Number of robots to spawn'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time_value,
        description='Use simulation time if true'
    )

    # Add declared launch arguments to the LaunchDescription
    ld.add_action(declare_turtlebot3_model_cmd)
    ld.add_action(declare_robot_count_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Launch Gazebo server and client
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'gzserver_args': '--factory'}.items()  # 使用 factory 启动
    )
    ld.add_action(gazebo_server)

    # Launch Gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    ld.add_action(gazebo_client)

    # Loop to spawn multiple robots
    for i in range(robot_count_value):
        namespace = f'robot_{i}'
        x_pose = str(-2.0 + i * 0.5)  # Adjust x pose
        y_pose = '-0.5'  # Keep y pose constant

        # Get the urdf file path
        model_folder = 'turtlebot3_' + turtlebot3_model_value
        urdf_path = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models',
            model_folder,
            'model.sdf'
        )

        # Create the spawn node for each robot
        spawn_turtlebot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', f'{turtlebot3_model_value}_{i}',  # Unique name for each robot
                '-file', urdf_path,
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01',
                '-robot_namespace', namespace,  # 使用正确的命名空间参数
            ],
            output='screen',
        )
        ld.add_action(spawn_turtlebot)

    return ld
