from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def make_node(context):
    overrides = {
        'mode': LaunchConfiguration('mode').perform(context),
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }
    for argument, parameter in (('map_directory', 'map.directory'), ('map_input_file', 'map.input_file')):
        value = LaunchConfiguration(argument).perform(context)
        if value:
            overrides[parameter] = value
    pose = LaunchConfiguration('initial_pose_xyz_xyzw').perform(context)
    if pose:
        overrides['initial_pose_xyz_xyzw'] = [float(value) for value in pose.split(',')]
    return [Node(
        package='lidar_mapping', executable='lidar_mapping_node', name='lidar_mapping',
        parameters=[LaunchConfiguration('config'), overrides], output='screen',
    )]


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('lidar_mapping'), 'config', 'lidar_mapping.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='gicp_mapping'),
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_directory', default_value=''),
        DeclareLaunchArgument('map_input_file', default_value=''),
        DeclareLaunchArgument('initial_pose_xyz_xyzw', default_value=''),
        OpaqueFunction(function=make_node),
    ])
