import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('autonomous_navigation'), 'config', 'trajectory_controller.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='map', description='Reference frame id'),
        DeclareLaunchArgument('duct_x', default_value='3.0', description='Duct position along the X-axis'),
        DeclareLaunchArgument('duct_y', default_value='1.5', description='Duct position along the Y-axis'),
        DeclareLaunchArgument('duct_z', default_value='-1.0', description='Duct position along the Z-axis'),
        DeclareLaunchArgument('delta', default_value='0.1', description='Distance between waypoints of the path'),
        DeclareLaunchArgument('side', default_value='2.15', description='Side length of the square'),
        DeclareLaunchArgument('radius', default_value='1.35', description='Radius of the circle or spiral'),
        DeclareLaunchArgument('dz', default_value='-1.0', description='Vertical step size of the spiral'),
        DeclareLaunchArgument('turns', default_value='4', description='Number of spiral'),
        DeclareLaunchArgument('depth', default_value='4', description='Depth of the serpetine path'),
        DeclareLaunchArgument('points', default_value='50', description='Number of points'),
        DeclareLaunchArgument('path_type', default_value='1', description='shape of the trajectory'),
        DeclareLaunchArgument('debug_mode', default_value='False', description='Write text logs to log_dir'),
        DeclareLaunchArgument('log_dir', default_value='/home/jetson/Documents/logs_auv',
                              description='Directory for the debug_mode text logs'),
        DeclareLaunchArgument('controller_params', default_value=default_params,
                              description='YAML with the PID gains / RC mapping of the trajectory controller'),
        DeclareLaunchArgument('namespace', default_value=['navigation'], description='Namespace'),

        Node(
            package='autonomous_navigation',
            executable='path_generator',
            name='path_generator',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{'frame_id': LaunchConfiguration('frame_id')}],
        ),

        Node(
            package='autonomous_navigation',
            executable='trajectory_controller',
            name='trajectory_controller',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('controller_params'),   # gains & RC mapping (file first, overrides below)
                {'frame_id': LaunchConfiguration('frame_id')},
                {'delta': LaunchConfiguration('delta')},
                {'side': LaunchConfiguration('side')},
                {'radius': LaunchConfiguration('radius')},
                {'dz': LaunchConfiguration('dz')},
                {'turns': LaunchConfiguration('turns')},
                {'depth': LaunchConfiguration('depth')},
                {'points': LaunchConfiguration('points')},
                {'path_type': LaunchConfiguration('path_type')},
                {'duct_x': LaunchConfiguration('duct_x')},
                {'duct_y': LaunchConfiguration('duct_y')},
                {'duct_z': LaunchConfiguration('duct_z')},
                {'debug_mode': LaunchConfiguration('debug_mode')},
                {'log_dir': LaunchConfiguration('log_dir')},
            ],
        ),
    ])
