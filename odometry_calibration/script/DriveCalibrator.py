#!/usr/bin/env python3
import threading
import time
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

MAX_POSES = 10

class DriveCalibrator(Node):
    def __init__(self):
        super().__init__('drive_calibrator')

        self.declare_parameter('camera_frame', 'c920_camera_link')

        self.camera_frame = self.get_parameter('camera_frame').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.odometry_pose_list = []


        # Start a background thread to listen to user input
        self.input_thread = threading.Thread(target=self._input_thread)
        self.input_thread.daemon = True
        self.input_thread.start()

        # Subscribe to the C920 raw image topic
        self.subscription = self.create_subscription(
            Odometry,
            '/wheel/odometry',
            self.odom_callback,
            10
        )

        self.pose_array_pub = self.create_publisher(
            PoseArray,
            '/calibrator/pose_array',
            10
        )

        # self.timer = self.create_timer(0.1, self.publish_pose_array)
        self.timer = self.create_timer(0.2, self.compute_pose)

        self.get_logger().info("Drive calibrator node started")

    def _input_thread(self):
        while True:
            user_input = input("Press 'p' to publish odometry poses as PoseArray: ")
            if user_input.lower() == 'p':
                self.publish_pose_array()
                self.get_logger().info("Published odometry poses as PoseArray.")

    
    def publish_pose_array(self):
        pose_array = PoseArray()
        pose_array.header.frame_id = 'world'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = self.odometry_pose_list
      
        self.pose_array_pub.publish(pose_array)

    def odom_callback(self, msg):
        pose = Pose()
        pose.position = msg.pose.pose.position
        pose.orientation = msg.pose.pose.orientation
        self.odometry_pose_list.append(pose)

        if len(self.odometry_pose_list) > MAX_POSES:
            self.odometry_pose_list.pop(0)

    # -----------------------------
    # Main computation
    # -----------------------------
    def compute_pose(self):

        self.get_logger().info(
            f"Current number of odometry poses stored: {len(self.odometry_pose_list)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DriveCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
