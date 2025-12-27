from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    
    # Load parameters from YAML files
    main_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'HILTI22.yaml'
    ])
    
    camera_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'camera_fisheye_HILTI22.yaml'
    ])
    
    # Main mapping node
    mapping_node = Node(
        package='fast_livo',
        executable='fastlivo_mapping',
        name='laserMapping',
        output='screen',
        parameters=[main_config, camera_config]
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
            'hilti.rviz'
        ])],
        condition=IfCondition(rviz)
    )
    
    return LaunchDescription([
        rviz_arg,
        mapping_node,
        rviz_node
    ])


