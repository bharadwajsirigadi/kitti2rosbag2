#!/usr/bin/env python3

import rclpy
import numpy as np
import message_filters
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from kitti_utils import KITTIOdometryDataset
from pathlib import Path
from nav_msgs.msg import Odometry

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

DATASET_DIR = "/media/psf/SSD/DRONES_LAB/kitti_dataset/dataset"
ODOM_DIR = '/media/psf/SSD/DRONES_LAB/kitti_dataset/dataset_2'
SEQUENCE = 0

class Kitti_Odom(Node):
    def __init__(self, data_dir: Path, odom_dir: Path, sequence: int):
        super().__init__("kitti_odom")
        self.kitti_dataset = KITTIOdometryDataset(data_dir, odom_dir, sequence)
        # self.timer = self.create_timer(0.5, self.send_velocity_command)
        self.left_imgs = self.kitti_dataset.left_images()
        self.right_imgs = self.kitti_dataset.right_images()
        self.counter = 0

        self.bridge = CvBridge()

        self.left_img_publisher_ = self.create_publisher(Image, '/camera1/left/image_raw', 10)
        self.right_img_publisher_ = self.create_publisher(Image, '/camera2/right/image_raw', 10)
        self.left_camera_info_publisher = self.create_publisher(CameraInfo, '/camera1/left/camera_info', 10)
        self.right_camera_info_publisher = self.create_publisher(CameraInfo, '/camera2/right/camera_info', 10)

        self.odom_publisher = self.create_publisher(Odometry,'/car_1/base/odom', 10)

    def publisher(self):
        left_image = cv2.imread(self.left_imgs[self.counter])
        right_image = cv2.imread(self.right_imgs[self.counter])
        left_img_msg = self.bridge.cv2_to_imgmsg(left_image, encoding='passthrough')
        right_img_msg = self.bridge.cv2_to_imgmsg(right_image, encoding='passthrough')
        self.get_logger().info("Published path")




        self.counter += 1
        return


def main(args=None):
    rclpy.init(args=args)
    node = Kitti_Odom()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()