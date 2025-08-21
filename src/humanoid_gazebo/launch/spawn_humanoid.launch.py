import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get the path to the humanoid_description package
    pkg_humanoid_description = get_package_share_directory('humanoid_description')
    
    # Path to the XACRO file
    xacro_file = os.path.join(pkg_humanoid_description, 'urdf', 'humanoid.urdf.xacro')
    
    # Path for the processed URDF file
    urdf_file = '/tmp/humanoid.urdf'
    
    # Generate URDF from XACRO
    os.system(f"xacro {xacro_file} > {urdf_file}")
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'verbose': 'true'}.items()
    )
    
    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'humanoid', '-file', urdf_file],
        output='screen'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file]
    )
    
    return LaunchDescription([
        gazebo,
        spawn_entity,
        robot_state_publisher
    ])