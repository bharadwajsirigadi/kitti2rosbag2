#!/usr/bin/env python3

import atexit
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String
import rosbag2_py
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from message_filters import TimeSynchronizer, Subscriber

class SimpleBagRecorder(Node):
    def __init__(self):
        self.counter = 0
        super().__init__('kitti_rec')
        self.get_logger().info("Node initialized")
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(uri='/home/parallels/Desktop/my_bag', storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        left_img_topic_info = rosbag2_py._storage.TopicMetadata(name='/camera1/left/image_raw', type='sensor_msgs/msg/Image', serialization_format='cdr')
        right_img_topic_info = rosbag2_py._storage.TopicMetadata(name='/camera2/right/image_raw', type='sensor_msgs/msg/Image', serialization_format='cdr')
        left_cam_info_topic_info = rosbag2_py._storage.TopicMetadata(name='/camera1/left/camera_info', type='sensor_msgs/msg/CameraInfo', serialization_format='cdr')
        right_cam_info_topic_info = rosbag2_py._storage.TopicMetadata(name='/camera2/right/camera_info', type='sensor_msgs/msg/CameraInfo', serialization_format='cdr')
        odom_topic_info = rosbag2_py._storage.TopicMetadata(name='/car_1/base/odom', type='nav_msgs/msg/Odometry', serialization_format='cdr')

        self.writer.create_topic(left_img_topic_info)
        self.writer.create_topic(right_img_topic_info)
        self.writer.create_topic(left_cam_info_topic_info)
        self.writer.create_topic(right_cam_info_topic_info)
        self.writer.create_topic(odom_topic_info)
        self.get_logger().info(f'Subscribed')

        left_img_sub = Subscriber(self, Image, '/camera1/left/image_raw')
        right_img_sub= Subscriber(self, Image, '/camera2/right/image_raw')
        # left_cam_info_sub = Subscriber(self, CameraInfo, '/camera1/left/camera_info')
        # right_cam_info_sub = Subscriber(self, CameraInfo, '/camera2/right/camera_info')
        odom_sub = Subscriber(self, Odometry, '/car_1/base/odom')

        ts = TimeSynchronizer([left_img_sub, right_img_sub, odom_sub], 10)
        self.get_logger().info("TimeSynchronizer created")

        ts.registerCallback(self.topic_callback)
        self.get_logger().info("Callback registered")

        # self.left_img_sub = self.create_subscription(Image, '/camera1/left/image_raw',)

        topics = {
            '/camera1/left/image_raw': Image,
            '/camera2/right/image_raw': Image,
            '/camera1/left/camera_info': CameraInfo,
            '/camera2/right/camera_info': CameraInfo,
            '/car_1/base/odom' : Odometry,
        }

        for topic, msg_type in topics.items():
            topic_info = rosbag2_py._storage.TopicMetadata(
                name=topic,
                type=f'{msg_type.__module__}/msg/{msg_type.__name__}',
                serialization_format='cdr')
            self.writer.create_topic(topic_info)

    def topic_callback(self, left_img_msg, right_img_msg, odom_msg):
        timestamp = self.get_clock().now().nanoseconds
        self.get_logger().info(f'Recorded')
        self.writer.write('/camera1/left/image_raw', left_img_msg, timestamp)
        self.writer.write('/camera2/right/image_raw', right_img_msg, timestamp)
        # self.writer.write('/camera1/left/camera_info', left_cam_info, timestamp)
        # self.writer.write('/camera2/right/camera_info', right_cam_info, timestamp)
        self.writer.write('/car_1/base/odom', odom_msg, timestamp)
        self.counter += 1

    def left_img_callback(self, msg):
        self.get_logger().info(f'Recorded')


    def cleanup(self):
        if self.writer.is_open():
            self.writer.close()

def main(args=None):
    rclpy.init(args=args)
    sbr = SimpleBagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()