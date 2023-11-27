#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, TransformStamped, Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
import tf2_ros
import cv2
import numpy as np
import message_filters
from kitti_odometry_bag_generator.utils.kitti_utils import KITTIOdometryDataset
from kitti_odometry_bag_generator.utils.quaternion import Quaternion
from tf2_ros import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
from tf2_geometry_msgs import do_transform_point, do_transform_vector3

DATASET_DIR = "/media/psf/SSD/DRONES_LAB/kitti_dataset/dataset"
ODOM_DIR = '/media/psf/SSD/DRONES_LAB/kitti_dataset/dataset_2'
SEQUENCE = 0

MAX_FRAMES = 1000 # Max frames in dataset
MIN_FEATURES = 50 # Minimum number of features for reliable relative pose between frames

class ComputeOdom():
    def __init__(self, data_dir, odom_dir, sequence):
        kitti_dataset = KITTIOdometryDataset(data_dir, odom_dir, sequence)
        self.p = kitti_dataset.projection_matrix(0)
        self.fc = self.p[0,0]
        print(self.fc)
        self.pp = (self.p[0,3], self.p[1,3])
        print(self.pp)
        pass

    def featureTracking(self, image_1, image_2, points_1):
        # Set Lucas-Kanade Params
        lk_params = dict( winSize  = (21,21),
                        maxLevel = 3,
                        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

        # Calculate Optical Flow
        points_2, status, err = cv2.calcOpticalFlowPyrLK(image_1, image_2, points_1, None, **lk_params)
        status = status.reshape(status.shape[0])
        points_1 = points_1[status==1]
        points_2 = points_2[status==1]

        # Return Tracked Points
        return points_1,points_2

    def featureDetection(self):
        # Detect FAST Features
        thresh = dict(threshold=25, nonmaxSuppression=True)
        fast = cv2.FastFeatureDetector_create(**thresh)
        return fast

    def getK(self):
        return self.p[:, :3]

    def monoVO(self, frame_0, frame_1 ,MIN_NUM_FEAT):
        # Input: Two image frames
        # Returns: Rotation matrix and translation vector, boolean validity, ignore pose if false
        image_1 = frame_0
        image_2 = frame_1

        detector = self.featureDetection()
        kp1 = detector.detect(image_1)
        points_1  = np.array([ele.pt for ele in kp1],dtype='float32')
        points_1, points_2 = self.featureTracking(image_1, image_2, points_1)

        K  = self.getK()

        E, mask = cv2.findEssentialMat(points_2, points_1, self.fc, self.pp, cv2.RANSAC,0.999,1.0)
        _, R, t, mask = cv2.recoverPose(E, points_2, points_1,focal=self.fc, pp = self.pp)

        if len(points_2) < MIN_NUM_FEAT:
            return R,t, False
        return R,t, True

class MonoVO(Node):
    def __init__(self):
        super().__init__("mono_node")
        self.bridge = CvBridge()
        self.left_img_sub = message_filters.Subscriber(self, Image, '/camera1/left/image_raw')
        self.right_img_sub = message_filters.Subscriber(self, Image, '/camera1/left/image_raw')
        ts = message_filters.TimeSynchronizer([self.left_img_sub, self.right_img_sub], 10)
        self.path_publisher = self.create_publisher(Path, '/car_1/path', 10)
        ts.registerCallback(self.callback)
        self.path_publisher = self.create_publisher(Path, '/car_1/path', 10)
        self.plot_Trajectory = self.create_publisher(Marker, "/kitti/pose", 10)
        self.prev_image = None
        self.min_features = 500
        self.p_msg = Path()
        self.translation_vector = np.array([[0], [0], [0]])
        self.rotation_matrix = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
        self.points = Marker()
        self.odom_publisher = self.create_publisher(Odometry, 'odometry_topic', 10)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        pass

    def callback(self, left_img_msg, right_img_msg):
        if self.prev_image is None:
            self.prev_image = left_img_msg
        else:
            Odometry = ComputeOdom(DATASET_DIR, ODOM_DIR, SEQUENCE)
            prev_image = self.bridge.imgmsg_to_cv2(self.prev_image)
            left_img = self.bridge.imgmsg_to_cv2(left_img_msg)

            r_Matrix, t_Vec, valid =  Odometry.monoVO(prev_image, left_img, self.min_features)
            if valid == True:
                self.translation_vector = self.translation_vector + self.rotation_matrix.dot(t_Vec)
                self.rotation_matrix = self.rotation_matrix.dot(r_Matrix)
                self.prev_image = left_img_msg                                                                                                                                                                                                                                 
                self.plotTraj(self.rotation_matrix, self.translation_vector)
        return

    def plotTraj(self, r_Matrix, t_Vec):
        r = np.eye(3) 
        t = t_Vec
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "map"  
        static_transform.child_frame_id = "odom" 
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        self.static_tf_broadcaster.sendTransform(static_transform)

        transformation_mtx = np.eye(4, dtype=float)
        transformation_mtx[:3, :3] = r_Matrix
        transformation_mtx[:3, 3] = t_Vec.flatten()
        # print(transformation_mtx)

        quaternion = Quaternion(transformation_mtx)
        x, y, z, w = quaternion.transformation_to_quaternion()

        pose = PoseStamped()
        pose.pose.position.x = t_Vec[0,0]
        pose.pose.position.y = t_Vec[1,0]
        # pose.pose.position.z = t_Vec[2,0]
        # pose.pose.orientation.x = x
        # pose.pose.orientation.y = y
        # pose.pose.orientation.z = z
        # pose.pose.orientation.w = w
        self.p_msg.poses.append(pose)
        self.p_msg.header.frame_id = "odom"
        # print(pose)
        self.path_publisher.publish(self.p_msg)

    def new_method(self):
        return 4
    
def main(args=None):
    rclpy.init(args=args)
    node = MonoVO()
    rclpy.spin(node)
    rclpy.shutdown()
    return

if __name__ == "__main__":
    main()



