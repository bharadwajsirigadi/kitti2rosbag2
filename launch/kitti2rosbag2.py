from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

pkg_name = 'kitti_odometry_bag_generator'
pkg_dir = os.popen('/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && \
                   colcon_cd %s && pwd"' % pkg_name).read().strip()

def generate_launch_description():

    kitti_dir_launch_arg = DeclareLaunchArgument(
        'kitti_dir',
        default_value='/media/psf/SSD/DRONES_LAB/kitti_dataset/dataset',
        description = 'Kitti Dataset directory'
    )
    odom_dir_launch_arg = DeclareLaunchArgument(
        'odom_dir',
        default_value='/media/psf/SSD/DRONES_LAB/kitti_dataset/dataset_2',
        description = 'Ground truth data directory'
    )
    sequence_launch_arg = DeclareLaunchArgument(
        'sequence',
        default_value='0',
        description = 'sequence'
    )
    bag_path_launch_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/default',
        description = 'Directory to save bag file'
    )

    kitti_sub = Node(
        package='kitti_odometry_bag_generator',
        namespace='',
        executable='kitti_sub',
        name='kitti_sub',
        # required = False
    )

    kitti_odom = Node(
        package='kitti_odometry_bag_generator',
        namespace='',
        executable='kitti_odom',
        name='kitti_odom',
        parameters=[
            # {'kitti_dir', LaunchConfiguration('kitti_dir')},
            # {'odom_dir', LaunchConfiguration('odom_dir')},
            {'sequence', LaunchConfiguration('sequence')}
            # {'bag_path', LaunchConfiguration('bag_path')}
        ],
        output = 'screen',
        # required = False
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=[os.path.join(get_package_share_directory('kitti_odometry_bag_generator'), 'rviz', 'kitti2rosbag2.rviz')],
        # required = False
    )

    return LaunchDescription([
        # kitti_dir_launch_arg,
        # odom_dir_launch_arg,
        sequence_launch_arg,
        # bag_path_launch_arg,
        kitti_sub,
        kitti_odom,
        rviz
    ])
    