from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
import launch.actions
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import pathlib
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Blue Dot Node
    blue_dot = Node(
            package='rallp', 
            executable='blue_dot_control.py', 
            output='screen', 
        )
    
    # Paths to URDF and RViz config
    pkg_share = FindPackageShare(package='rallp').find('rallp')
    default_model_path = os.path.join(pkg_share, 'urdf', 'rallp3.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'config2.rviz')
    bridge_params = os.path.join(get_package_share_directory('rallp'),'config','gz_bridge.yaml')
   

    # Load the URDF file as a string
    with open(default_model_path, 'r') as f:
        robot_description_content = f.read()

    # Wrap the URDF content in ParameterValue
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', '/home/ahmed-jeljli/rallp_v2.1/src/rallp/world/my_world.sdf'], # !!! NEED TO CHANGE THE DIRECTORY !!!
        output='screen'
    )

    # Spawn node
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-entity', 'rallp2',
            '-file', default_model_path,  
            '-z', '1'
        ],
        output='screen'
    )

    # ros_gz_bridge node
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',

        ],
        output='screen'
    )

    # Extended Kalman Filter node 
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("robot_localization"), 'config', 'ekf.yaml')],

    )
    


    
    return LaunchDescription([
        blue_dot,
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        robot_state_publisher_node,
        rviz_node,
        gazebo,
        spawn_entity,
        gazebo_bridge,
        #ekf_node,
    ])