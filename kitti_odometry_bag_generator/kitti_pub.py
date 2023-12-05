#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, Point, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from kitti_odometry_bag_generator.utils.kitti_utils import KITTIOdometryDataset
from kitti_odometry_bag_generator.utils.quaternion import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class Kitti_Odom(Node):
    def __init__(self):
        super().__init__("kitti_pub")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sequence', rclpy.Parameter.Type.INTEGER),
                ('data_dir', rclpy.Parameter.Type.STRING),
                ('odom', rclpy.Parameter.Type.BOOL),
                ('odom_dir', rclpy.Parameter.Type.STRING)
            ]
        )

        sequence = self.get_parameter('sequence').value
        self.get_logger().info(f'SEQUENCE:={sequence}')

        data_dir = self.get_parameter('data_dir').get_parameter_value().string_value
        self.get_logger().info(f'DATA_DIR:={data_dir}')

        odom = self.get_parameter('odom').value
        self.get_logger().info(f'ODOM:={odom}')
  
        if odom == True:
            odom_dir = self.get_parameter('odom_dir').get_parameter_value().string_value
            self.get_logger().info(f'ODOM_DIR:={odom_dir}')
        else:
            odom_dir = None

        self.kitti_dataset = KITTIOdometryDataset(data_dir, sequence, odom_dir)
        self.bridge = CvBridge()
        # self.get_logger().info('came here')
        self.counter = 0
        self.counter_limit = len(self.kitti_dataset.left_images()) - 1 
        
        self.left_imgs = self.kitti_dataset.left_images()
        self.right_imgs = self.kitti_dataset.right_images()
        self.times_file = self.kitti_dataset.times_file()
        self.odom_trigger = odom
        if self.odom_trigger == True:
            try:
                self.ground_truth = self.kitti_dataset.odom_pose()
            except FileNotFoundError as filenotfounderror:
                self.get_logger().error("Error: {}".format(filenotfounderror))
                rclpy.shutdown()
                return

        self.timer = self.create_timer(0.05, self.publish_callback)
        self.left_img_publisher_ = self.create_publisher(Image, '/camera1/left/image_raw', 10)
        self.right_img_publisher_ = self.create_publisher(Image, '/camera2/right/image_raw', 10)
        self.left_camera_info_publisher = self.create_publisher(CameraInfo, '/camera1/left/camera_info', 10)
        self.right_camera_info_publisher = self.create_publisher(CameraInfo, '/camera2/right/camera_info', 10)
        self.odom_publisher = self.create_publisher(Odometry,'/car_1/base/odom', 10)
        self.path_publisher = self.create_publisher(Path, '/car_1/path', 10)

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

    def publish_callback(self):
        
        # retrieving images and creating msg
        left_image = cv2.imread(self.left_imgs[self.counter])
        right_image = cv2.imread(self.right_imgs[self.counter])
        left_img_msg = self.bridge.cv2_to_imgmsg(left_image, encoding='passthrough')
        right_img_msg = self.bridge.cv2_to_imgmsg(right_image, encoding='passthrough')

        # retrieving clock_time and creating msg
        clock_time = self.times_file[self.counter]
        odom_msg = Odometry()
        odom_msg.header.stamp.sec = int(clock_time)
        odom_msg.header.stamp.nanosec = int((clock_time - int(clock_time)) * 1e9)

        # retrieving and creating ground truth msg
        if self.odom_trigger == True:

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
            self.odom_publisher.publish(odom_msg)

        # self.get_logger().info(f'{self.counter}-Images Processed')

        if self.counter >= self.counter_limit:
            self.get_logger().info('All images and poses published. Stopping...')
            rclpy.shutdown()
            self.timer.cancel()
        
        self.left_img_publisher_.publish(left_img_msg)
        self.right_img_publisher_.publish(right_img_msg)
        

        # Broadcast the static transform
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "map" 
        static_transform.child_frame_id = "odom"
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        self.static_tf_broadcaster.sendTransform(static_transform)

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