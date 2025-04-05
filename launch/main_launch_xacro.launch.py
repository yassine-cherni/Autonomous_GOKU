from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
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
    default_model_path = os.path.join(pkg_share, 'urdf', 'main.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'config2.rviz')
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')

    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("rallp"),'launch','robot_state_publisher.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
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
        cmd=['gz', 'sim', '-v', '4', '-r', '/home/ahmed-jeljli/rallp_v2.1/src/rallp/world/my_world.sdf'],
        output='screen'
    )

    # Spawn node
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.1'],
                        output='screen')

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
        rsp,
        rviz_node,
        gazebo,
        spawn_entity,
        gazebo_bridge,
    ])