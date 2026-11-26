from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare('lidar_mapping')
    config = PathJoinSubstitution([package, 'config', 'lidar_mapping.yaml'])
    mapping_launch = PathJoinSubstitution([package, 'launch', 'mapping.launch.py'])
    arguments = {
        'config': config,
        'use_sim_time': 'true',
        'map_directory': '',
        'map_input_file': '',
        'initial_pose_xyz_xyzw': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in arguments.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                'mode': 'gicp_localization',
                **{name: LaunchConfiguration(name) for name in arguments},
            }.items(),
        ),
    ])
