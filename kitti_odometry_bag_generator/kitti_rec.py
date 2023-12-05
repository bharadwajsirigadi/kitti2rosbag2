#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry, Path

class BagRecorder(Node):
    def __init__(self):
        self.counter = 0
        super().__init__('kitti_rec')
        self.get_logger().info("Node initialized")
        self.writer = rosbag2_py.SequentialWriter()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('bag_dir', rclpy.Parameter.Type.STRING),
            ]
        )
        bag_dir = self.get_parameter('bag_dir').get_parameter_value().string_value
        self.get_logger().info(f'BAG_DIR:={bag_dir}')
        if os.path.exists(bag_dir):
            self.get_logger().info(f'The directory {bag_dir} already exists. Shutting down...')
            rclpy.shutdown()
        else:
            storage_options = rosbag2_py._storage.StorageOptions(uri=bag_dir, storage_id='sqlite3')
            converter_options = rosbag2_py._storage.ConverterOptions('', '')
            self.writer.open(storage_options, converter_options)

        left_img_topic_info = rosbag2_py._storage.TopicMetadata(name='/camera1/left/image_raw', type='sensor_msgs/msg/Image', serialization_format='cdr')
        right_img_topic_info = rosbag2_py._storage.TopicMetadata(name='/camera2/right/image_raw', type='sensor_msgs/msg/Image', serialization_format='cdr')
        odom_path_topic_info = rosbag2_py._storage.TopicMetadata(name='/car_1/odom_path', type='nav_msgs/msg/Path', serialization_format='cdr')
        left_cam_info_topic_info = rosbag2_py._storage.TopicMetadata(name='/camera1/left/camera_info', type='sensor_msgs/msg/CameraInfo', serialization_format='cdr')
        right_cam_info_topic_info = rosbag2_py._storage.TopicMetadata(name='/camera2/right/camera_info', type='sensor_msgs/msg/CameraInfo', serialization_format='cdr')
        odom_topic_info = rosbag2_py._storage.TopicMetadata(name='/car_1/base/odom', type='nav_msgs/msg/Odometry', serialization_format='cdr')

        self.writer.create_topic(left_img_topic_info)
        self.writer.create_topic(right_img_topic_info)
        self.writer.create_topic(odom_path_topic_info)
        self.writer.create_topic(left_cam_info_topic_info)
        self.writer.create_topic(right_cam_info_topic_info)
        self.writer.create_topic(odom_topic_info)
        
        self.left_img_sub = self.create_subscription(Image, '/camera1/left/image_raw', self.left_img_callback, 10)
        self.right_img_sub = self.create_subscription(Image, '/camera2/right/image_raw', self.right_img_callback, 10)
        self.odom_path_sub = self.create_subscription(Path, 'car_1/odom_path', self.odom_path_callback, 10)
        self.left_cam_info_sub = self.create_subscription(CameraInfo, '/camera1/left/camera_info', self.left_cam_info_callback, 10)
        self.right_cam_info_sub = self.create_subscription(CameraInfo, '/camera2/right/camera_info', self.right_cam_info_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/car_1/base/odom', self.odom_callback, 10)

    def left_img_callback(self, msg):
        timestamp = self.get_clock().now().nanoseconds
        self.writer.write('/camera1/left/image_raw', serialize_message(msg), timestamp)
        self.get_logger().info(f'Recorded-{self.counter}')
        self.counter += 1

    def right_img_callback(self, msg):
        timestamp = self.get_clock().now().nanoseconds
        self.writer.write('/camera2/right/image_raw', serialize_message(msg), timestamp)
        # self.get_logger().info(f'Recorded right_img')

    def odom_path_callback(self, msg):
        timestamp = self.get_clock().now().nanoseconds
        self.writer.write('/car_1/odom_path', serialize_message(msg), timestamp)
        # self.get_logger().info(f'Recorded odom_path')

    def left_cam_info_callback(self, msg):
        timestamp = self.get_clock().now().nanoseconds
        self.writer.write('/camera1/left/camera_info', serialize_message(msg), timestamp)
        # self.get_logger().info(f'Recorded left_cam_info')

    def right_cam_info_callback(self, msg):
        timestamp = self.get_clock().now().nanoseconds
        self.writer.write('/camera2/right/camera_info', serialize_message(msg), timestamp)
        # self.get_logger().info(f'Recorded right_cam_info')

    def odom_callback(self, msg):
        timestamp = self.get_clock().now().nanoseconds
        self.writer.write('/car_1/base/odom', serialize_message(msg), timestamp)
        # self.get_logger().info(f'Recorded odom')

    def cleanup(self):
        if self.writer.is_open():
            self.writer.close()

def main(args=None):
    rclpy.init(args=args)
    sbr = BagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()