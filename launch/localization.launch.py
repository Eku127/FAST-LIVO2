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

    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='',
        description='Path to the pre-built voxel map file'
    )

    init_x_arg = DeclareLaunchArgument(
        'init_x',
        default_value='0.0',
        description='Initial X position'
    )

    init_y_arg = DeclareLaunchArgument(
        'init_y',
        default_value='0.0',
        description='Initial Y position'
    )

    init_z_arg = DeclareLaunchArgument(
        'init_z',
        default_value='0.0',
        description='Initial Z position'
    )

    init_roll_arg = DeclareLaunchArgument(
        'init_roll',
        default_value='0.0',
        description='Initial roll angle (radians)'
    )

    init_pitch_arg = DeclareLaunchArgument(
        'init_pitch',
        default_value='0.0',
        description='Initial pitch angle (radians)'
    )

    init_yaw_arg = DeclareLaunchArgument(
        'init_yaw',
        default_value='0.0',
        description='Initial yaw angle (radians)'
    )

    map_update_mode_arg = DeclareLaunchArgument(
        'map_update_mode',
        default_value='none',
        description='Map update mode: none / incremental / full'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='mid360.yaml',
        description='Configuration file name in config directory'
    )

    # Get launch configurations
    rviz = LaunchConfiguration('rviz')
    map_path = LaunchConfiguration('map_path')
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_z = LaunchConfiguration('init_z')
    init_roll = LaunchConfiguration('init_roll')
    init_pitch = LaunchConfiguration('init_pitch')
    init_yaw = LaunchConfiguration('init_yaw')
    map_update_mode = LaunchConfiguration('map_update_mode')
    config_file = LaunchConfiguration('config_file')

    # Load parameters from YAML file
    config_path = PathJoinSubstitution([
        pkg_share,
        'config',
        config_file
    ])

    # Main localization node
    localization_node = Node(
        package='fast_livo',
        executable='fastlivo_mapping',
        name='fastlivo_localization',
        output='screen',
        parameters=[
            config_path,
            {
                'relocalization.enabled': True,
                'relocalization.prior_map_path': map_path,
                'relocalization.map_update_mode': map_update_mode,
                'relocalization.initial_pose.x': init_x,
                'relocalization.initial_pose.y': init_y,
                'relocalization.initial_pose.z': init_z,
                'relocalization.initial_pose.roll': init_roll,
                'relocalization.initial_pose.pitch': init_pitch,
                'relocalization.initial_pose.yaw': init_yaw,
            }
        ]
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
        map_path_arg,
        init_x_arg,
        init_y_arg,
        init_z_arg,
        init_roll_arg,
        init_pitch_arg,
        init_yaw_arg,
        map_update_mode_arg,
        config_file_arg,
        localization_node,
        rviz_node
    ])
