from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import os

package_name = 'kitti2rosbag2'

def generate_launch_description():

    params_config = os.path.join(
        get_package_share_directory(package_name),
        'params.yaml'
    )

    rviz_config = os.path.join(
        get_package_share_directory(package_name), 
        'rviz', 
        'kitti2rosbag2.rviz'
    )

    node = Node(
        package=package_name,
        namespace='',
        executable='kitti_pub_node',
        name='kitti_pub',
        parameters=[params_config],
    )

    kitti_sub = Node(
        package=package_name,
        namespace='',
        executable='kitti_sub',
        name='kitti_sub',
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        parameters = [rviz_config]
    )

    kitti_rec = Node(
        package=package_name,
        namespace='',
        executable='kitti_rec',
        name='kitti_rec',
        parameters=[params_config]
    )

    return LaunchDescription([
        kitti_sub,
        node,
        rviz,
        kitti_rec,
    ])