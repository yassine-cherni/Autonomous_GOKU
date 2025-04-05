from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    # Blue Dot Node
    blue_dot = Node(
        package='rallp', 
        executable='blue_dot_control.py', 
        output='screen', 
    )
    
    # Paths to URDF and RViz config
    pkg_share = FindPackageShare(package='rallp').find('rallp')
    default_model_path = os.path.join(pkg_share, 'urdf', 'rallp.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'config2.rviz')
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')

    # Generate URDF from XACRO

    robot_description_content = Command(['xacro', ' ', default_model_path])

    # Robot description for robot_state_publisher
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': True}],
    )

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', PathJoinSubstitution([pkg_share, 'world', 'my_world.sdf'])],
        output='screen'
    )

    # Spawn node
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-entity', 'rallp2',
            '-topic', '/robot_description',
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

    return LaunchDescription([
        blue_dot,
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        robot_state_publisher_node,
        rviz_node,
        gazebo,
        spawn_entity,
        gazebo_bridge,
    ])