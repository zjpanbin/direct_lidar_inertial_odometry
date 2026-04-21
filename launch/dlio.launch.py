# launch/dlio.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('direct_lidar_inertial_odometry')

    dlio_yaml = os.path.join(pkg_share, 'cfg', 'dlio.yaml')
    params_yaml = os.path.join(pkg_share, 'cfg', 'params.yaml')
    rviz_cfg = os.path.join(pkg_share, 'launch', 'dlio.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='robot'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('pointcloud_topic', default_value='lidar'),
        DeclareLaunchArgument('imu_topic', default_value='imu'),

        Node(
            package='direct_lidar_inertial_odometry',
            executable='dlio_odom_node',
            name='dlio_odom',
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen',
            parameters=[dlio_yaml, params_yaml],
            remappings=[
                ('pointcloud', LaunchConfiguration('pointcloud_topic')),
                ('imu', LaunchConfiguration('imu_topic')),
                ('odom', 'dlio/odom_node/odom'),
                ('pose', 'dlio/odom_node/pose'),
                ('path', 'dlio/odom_node/path'),
                ('kf_pose', 'dlio/odom_node/keyframes'),
                ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
                ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
            ],
        ),

        Node(
            package='direct_lidar_inertial_odometry',
            executable='dlio_map_node',
            name='dlio_map',
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen',
            parameters=[dlio_yaml, params_yaml],
            remappings=[
                ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
                ('map', 'dlio/map_node/map'),
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='dlio_rviz',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', rviz_cfg],
        ),
    ])
