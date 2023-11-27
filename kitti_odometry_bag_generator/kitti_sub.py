#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class kitti_sub(Node):
    def __init__(self):
        super().__init__("kitti_sub")
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.odom_sub = self.create_subscription(Odometry, '/car_1/base/odom', self.subscribe_callback, qos_profile)
        self.path_publisher = self.create_publisher(Path, '/car_1/path', 10)
        self.p_msg = Path()

    def subscribe_callback(self, odom_msg):
        
        print("called")
        pose= PoseStamped()
        pose.pose = odom_msg.pose.pose
        pose.header.frame_id = "odom"
        self.p_msg.poses.append(pose)
        self.p_msg.header.frame_id = "map"
        self.path_publisher.publish(self.p_msg)
        return

def main(args=None):
    rclpy.init(args=args)
    node = kitti_sub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()