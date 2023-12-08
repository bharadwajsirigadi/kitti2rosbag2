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

    kitti_rec = Node(
        package=package_name,
        namespace='',
        executable='kitti_rec_node',
        name='kitti_rec',
        parameters=[params_config],
        output='screen'
    )

    return LaunchDescription([
        kitti_rec,
    ])