#!/usr/bin/env python3

import rclpy
import numpy as np
import message_filters
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, Point, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from kitti_odometry_bag_generator.utils.kitti_utils import KITTIOdometryDataset
from kitti_odometry_bag_generator.utils.quaternion import Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf2_ros import TransformListener, Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

DATASET_DIR = "/media/psf/SSD/DRONES_LAB/kitti_dataset/dataset"
ODOM_DIR = '/media/psf/SSD/DRONES_LAB/kitti_dataset/dataset_2'
SEQUENCE = 0

class Kitti_Odom(Node):
    def __init__(self, data_dir, odom_dir, sequence: int):
        super().__init__("kitti_odom")
        self.kitti_dataset = KITTIOdometryDataset(data_dir, odom_dir, sequence)
        self.bridge = CvBridge()

        self.counter = 0
        self.counter_limit = 4540
        
        self.left_imgs = self.kitti_dataset.left_images()
        self.right_imgs = self.kitti_dataset.right_images()
        self.times_file = self.kitti_dataset.times_file()
        self.ground_truth = self.kitti_dataset.odom_pose()

        self.timer = self.create_timer(0.05, self.publish_callback)
        self.left_img_publisher_ = self.create_publisher(Image, '/camera1/left/image_raw', 10)
        self.right_img_publisher_ = self.create_publisher(Image, '/camera2/right/image_raw', 10)
        self.left_camera_info_publisher = self.create_publisher(CameraInfo, '/camera1/left/camera_info', 10)
        self.right_camera_info_publisher = self.create_publisher(CameraInfo, '/camera2/right/camera_info', 10)
        self.odom_publisher = self.create_publisher(Odometry,'/car_1/base/odom', 10)
        self.path_publisher = self.create_publisher(Path, '/car_1/path', 10)

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

    def rotationMatrixToQuaternion(self, m):
        t = np.matrix.trace(m)
        q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

        if(t > 0):
            t = np.sqrt(t + 1)
            q[0] = 0.5 * t
            t = 0.5/t
            q[1] = (m[2,1] - m[1,2]) * t
            q[2] = (m[0,2] - m[2,0]) * t
            q[3] = (m[1,0] - m[0,1]) * t

        else:
            i = 0
            if (m[1,1] > m[0,0]):
                i = 1
            if (m[2,2] > m[i,i]):
                i = 2
            j = (i+1)%3
            k = (j+1)%3

            t = np.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
            q[i] = 0.5 * t
            t = 0.5 / t
            q[0] = (m[k,j] - m[j,k]) * t
            q[j] = (m[j,i] + m[i,j]) * t
            q[k] = (m[k,i] + m[i,k]) * t

        return q

    def publish_callback(self):
        # retrieving images and creating msg
        left_image = cv2.imread(self.left_imgs[self.counter])
        right_image = cv2.imread(self.right_imgs[self.counter])
        left_img_msg = self.bridge.cv2_to_imgmsg(left_image, encoding='bgr8')
        right_img_msg = self.bridge.cv2_to_imgmsg(right_image, encoding='bgr8')

        # retrieving clock_time and creating msg
        clock_time = self.times_file[self.counter]
        odom_msg = Odometry()
        odom_msg.header.stamp.sec = int(clock_time)
        odom_msg.header.stamp.nanosec = int((clock_time - int(clock_time)) * 1e9)

        # retrieving and creating ground truth msg
        quaternion = Quaternion(self.ground_truth[self.counter])
        x, y, z, w = quaternion.transformation_to_quaternion()
        translation = self.ground_truth[self.counter][:3,3]
        a = self.rotationMatrixToQuaternion(self.ground_truth[self.counter][:3, :3])
        # print(a)
        # odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.z = -translation[1]
        odom_msg.pose.pose.position.y = -translation[2] 
        odom_msg.pose.pose.position.x = -translation[0] 
        odom_msg.pose.pose.orientation.x = a[0]
        odom_msg.pose.pose.orientation.y = a[1]
        odom_msg.pose.pose.orientation.z = a[2]
        odom_msg.pose.pose.orientation.w = a[3]

        self.get_logger().info(f'{self.counter}-Images Processed')

        if self.counter >= self.counter_limit:
            self.get_logger().info('All images and poses published. Stopping...')
            self.timer.cancel()
        
        self.left_img_publisher_.publish(left_img_msg)
        self.right_img_publisher_.publish(right_img_msg)
        self.odom_publisher.publish(odom_msg)

        # Broadcast the static transform (adjust the values as needed)
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "map"  # Set your fixed frame
        static_transform.child_frame_id = "odom"  # Set your odometry frame
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        # static_transform.transform.rotation.x = 0.0
        # static_transform.transform.rotation.y = 0.0
        # static_transform.transform.rotation.z = 0.0
        # static_transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(static_transform)

        self.counter += 1
        return

def main(args=None):
    rclpy.init(args=args)
    node = Kitti_Odom(DATASET_DIR, ODOM_DIR, SEQUENCE)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()