from setuptools import find_packages, setup
from glob import glob

package_name = 'kitti_odometry_bag_generator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/kitti2rosbag2.launch.py']),
        ('share/' + package_name, ['config/params.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bharadwajsirigadi',
    maintainer_email='bharadwajsirigadi@gmail.com',
    description='KITTI odometry to ROS2 bag converter',
    license='Apache License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "kitti_pub_node = kitti_odometry_bag_generator.kitti_pub:main",
            "kitti_sub = kitti_odometry_bag_generator.kitti_sub:main",
            "mono_node = kitti_odometry_bag_generator.mono_vo:main",
            "kitti_rec = kitti_odometry_bag_generator.kitti_rec:main"
        ],
    },
)
