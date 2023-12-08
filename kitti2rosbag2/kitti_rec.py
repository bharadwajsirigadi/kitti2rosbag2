#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2
import rosbag2_py
import os
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from kitti2rosbag2.utils.kitti_utils import KITTIOdometryDataset
from kitti2rosbag2.utils.quaternion import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.serialization import serialize_message
from rclpy.time import Time

class Kitti_Odom(Node):
    def __init__(self):
        super().__init__("kitti_rec")

        # PARAMETERS
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sequence', rclpy.Parameter.Type.INTEGER),
                ('data_dir', rclpy.Parameter.Type.STRING),
                ('odom', rclpy.Parameter.Type.BOOL),
                ('odom_dir', rclpy.Parameter.Type.STRING),
                ('bag_dir', rclpy.Parameter.Type.STRING),
            ]
        )

        sequence = self.get_parameter('sequence').value
        data_dir = self.get_parameter('data_dir').get_parameter_value().string_value
        odom = self.get_parameter('odom').value
        bag_dir = self.get_parameter('bag_dir').get_parameter_value().string_value
        if odom == True:
            odom_dir = self.get_parameter('odom_dir').get_parameter_value().string_value
        else:
            odom_dir = None
        
        self.kitti_dataset = KITTIOdometryDataset(data_dir, sequence, odom_dir)
        self.bridge = CvBridge()
        self.counter = 0
        self.counter_limit = len(self.kitti_dataset.left_images()) - 1 
        
        self.left_imgs = self.kitti_dataset.left_images()
        self.right_imgs = self.kitti_dataset.right_images()
        self.times_file = self.kitti_dataset.times_file()
        self.odom = odom
        if odom == True:
            try:
                self.ground_truth = self.kitti_dataset.odom_pose()
            except FileNotFoundError as filenotfounderror:
                self.get_logger().error("Error: {}".format(filenotfounderror))
                rclpy.shutdown()
                return

        # ROSBAG WRITER
        self.writer = rosbag2_py.SequentialWriter()
        if os.path.exists(bag_dir):
            self.get_logger().info(f'The directory {bag_dir} already exists. Shutting down...')
            rclpy.shutdown()
        else:
            storage_options = rosbag2_py._storage.StorageOptions(uri=bag_dir, storage_id='sqlite3')
            converter_options = rosbag2_py._storage.ConverterOptions('', '')
            self.writer.open(storage_options, converter_options)

        left_img_topic_info = rosbag2_py._storage.TopicMetadata(name='/camera1/left/image_raw', type='sensor_msgs/msg/Image', serialization_format='cdr')
        right_img_topic_info = rosbag2_py._storage.TopicMetadata(name='/camera2/right/image_raw', type='sensor_msgs/msg/Image', serialization_format='cdr')
        odom_topic_info = rosbag2_py._storage.TopicMetadata(name='/car_1/base/odom', type='nav_msgs/msg/Odometry', serialization_format='cdr')

        self.writer.create_topic(left_img_topic_info)
        self.writer.create_topic(right_img_topic_info)
        self.writer.create_topic(odom_topic_info)

        # PERKS
        self.timer = self.create_timer(0.05, self.publish_callback)

    def publish_callback(self):
        time = self.times_file[self.counter]
        timestamp = int((time) * 1e9)

        # retrieving images and creating msg
        left_image = cv2.imread(self.left_imgs[self.counter])
        right_image = cv2.imread(self.right_imgs[self.counter])
        left_img_msg = self.bridge.cv2_to_imgmsg(left_image, encoding='passthrough')
        right_img_msg = self.bridge.cv2_to_imgmsg(right_image, encoding='passthrough')

        odom_msg = Odometry()
        if self.odom == True:
            translation = self.ground_truth[self.counter][:3,3]
            quaternion = Quaternion()
            a = quaternion.rotationmtx_to_quaternion(self.ground_truth[self.counter][:3, :3])
            odom_msg.pose.pose.position.z = -translation[1]
            odom_msg.pose.pose.position.y = -translation[2] 
            odom_msg.pose.pose.position.x = -translation[0] 
            odom_msg.pose.pose.orientation.x = a[0]
            odom_msg.pose.pose.orientation.y = a[1]
            odom_msg.pose.pose.orientation.z = a[2]
            odom_msg.pose.pose.orientation.w = a[3]
            self.writer.write('/car_1/base/odom', serialize_message(odom_msg), timestamp)


        # Recording Bag
        self.writer.write('/camera1/left/image_raw', serialize_message(left_img_msg), timestamp)
        self.writer.write('/camera2/right/image_raw', serialize_message(right_img_msg), timestamp)

        self.get_logger().info(f'{self.counter}-Images Processed')
        if self.counter >= self.counter_limit:
            self.get_logger().info('All images and poses published. Stopping...')
            rclpy.shutdown()
            self.timer.cancel()
        self.counter += 1
        return

def main(args=None):
    rclpy.init(args=args)
    node = Kitti_Odom()
    rclpy.spin(node)
    try:
        rclpy.shutdown()
    except Exception as e:
        pass

if __name__ == '__main__':
    main()