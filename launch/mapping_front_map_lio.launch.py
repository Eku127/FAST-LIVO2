from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('fast_livo').find('fast_livo')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RViz'
    )
    rviz = LaunchConfiguration('rviz')

    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'front_map_lio.yaml'
    ])

    mapping_node = Node(
        package='fast_livo',
        executable='fastlivo_mapping',
        name='laserMapping',
        output='screen',
        parameters=[config_file]
    )

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
