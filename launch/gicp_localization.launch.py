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
    prior_pcd_arg = DeclareLaunchArgument(
        'prior_pcd_path',
        default_value='',
        description='Path to prior PCD map file'
    )

    search_radius_arg = DeclareLaunchArgument(
        'search_radius',
        default_value='5.0',
        description='Local map extraction radius (meters)'
    )

    downsampling_resolution_arg = DeclareLaunchArgument(
        'downsampling_resolution',
        default_value='0.1',
        description='Voxel grid downsampling resolution (meters)'
    )

    max_correspondence_distance_arg = DeclareLaunchArgument(
        'max_correspondence_distance',
        default_value='1.0',
        description='Maximum correspondence distance for GICP (meters)'
    )

    max_iterations_arg = DeclareLaunchArgument(
        'max_iterations',
        default_value='20',
        description='Maximum number of GICP iterations'
    )

    num_threads_arg = DeclareLaunchArgument(
        'num_threads',
        default_value='4',
        description='Number of threads for GICP'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    # GICP localization node
    gicp_loc_node = Node(
        package='fast_livo',
        executable='gicp_localization_node',
        name='gicp_localization_node',
        output='screen',
        parameters=[{
            'prior_pcd_path': LaunchConfiguration('prior_pcd_path'),
            'search_radius': LaunchConfiguration('search_radius'),
            'downsampling_resolution': LaunchConfiguration('downsampling_resolution'),
            'max_correspondence_distance': LaunchConfiguration('max_correspondence_distance'),
            'max_iterations': LaunchConfiguration('max_iterations'),
            'num_threads': LaunchConfiguration('num_threads'),
            'publish_local_map': True,
            'verbose': True,
        }]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_gicp_localization',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            pkg_share,
            'rviz_cfg',
            'fast_livo2_local.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        prior_pcd_arg,
        search_radius_arg,
        downsampling_resolution_arg,
        max_correspondence_distance_arg,
        max_iterations_arg,
        num_threads_arg,
        rviz_arg,
        gicp_loc_node,
        rviz_node,
    ])
