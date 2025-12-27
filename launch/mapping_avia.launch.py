from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = FindPackageShare('fast_livo').find('fast_livo')
    
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    # Get launch configuration
    rviz = LaunchConfiguration('rviz')
    
    # Load parameters from YAML file
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'avia.yaml'
    ])
    
    # Main mapping node
    mapping_node = Node(
        package='fast_livo',
        executable='fastlivo_mapping',
        name='laserMapping',
        output='screen',
        parameters=[config_file]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            pkg_share,
            'rviz_cfg',
            'fast_livo2.rviz'
        ])],
        condition=IfCondition(rviz)
    )
    
    return LaunchDescription([
        rviz_arg,
        mapping_node,
        rviz_node
    ])

