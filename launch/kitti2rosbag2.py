
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


pkg_name = 'kitti_odometry_bag_generator'
pkg_dir = os.popen('/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && \
                   colcon_cd %s && pwd"' % pkg_name).read().strip()

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kitti_odometry_bag_generator',
            namespace='',
            executable='kitti_odom',
            name='sim'
        ),
        Node(
            package='kitti_odometry_bag_generator',
            namespace='',
            executable='kitti_sub',
            name='sim'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=[os.path.join(get_package_share_directory('kitti_odometry_bag_generator'), 'rviz', 'kitti2rosbag2.rviz')]
        )
    ])