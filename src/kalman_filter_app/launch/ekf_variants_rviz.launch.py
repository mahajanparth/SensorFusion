"""Runs EKF variants A, B and C side by side and shows them in RViz2 against GPS.

    ros2 launch kalman_filter_app ekf_variants_rviz.launch.py alignment_file:=<repo>/eval/results/alignment.json
    ros2 bag play <bag_dir> --topics /wheel_odom /imu/data_raw /fix
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('kalman_filter_app')

    variants = [
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'launch_ekf_node.py')),
            launch_arguments={
                'variant': v,
                'node_name': f'ekf_{v}',
                'output_topic': f'/ekf_{v}/odom',
            }.items(),
        )])
        for v in ('A', 'B', 'C')
    ]

    return LaunchDescription([
        DeclareLaunchArgument('alignment_file', default_value=os.path.join(os.getcwd(), 'eval', 'results', 'alignment.json'),
                              description='SE(2) alignments from eval/evaluate.py (sources drawn in their own frames if missing)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        *variants,
        Node(package='kalman_filter_app', executable='variant_path_publisher.py', output='screen',
             parameters=[{'alignment_file': LaunchConfiguration('alignment_file')}]),
        # Gives RViz a "map" frame to use as its fixed frame.
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'gps_enu']),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(share, 'rviz', 'ekf_variants.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
