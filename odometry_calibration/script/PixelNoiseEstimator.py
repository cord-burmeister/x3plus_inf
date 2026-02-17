#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge


class PixelNoiseNode(Node):
    def __init__(self):
        super().__init__('pixel_noise_node')

        self.bridge = CvBridge()

        # Subscribe to the C920 raw image topic
        self.subscription = self.create_subscription(
            Image,
            '/c920_camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info("Pixel Noise Node started. Listening to /c920_camera/image_raw")

    def image_callback(self, msg):
        # Convert ROS Image → OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale for simpler noise metrics
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Compute noise as standard deviation of pixel intensities
        noise_std = float(np.std(gray))

        # Optional: per-channel noise
        b_std = float(np.std(frame[:, :, 0]))
        g_std = float(np.std(frame[:, :, 1]))
        r_std = float(np.std(frame[:, :, 2]))

        if (noise_std < 8.0):

            self.get_logger().info(
                f"Noise (gray std): {noise_std:.3f} | "
                f"B: {b_std:.3f}, G: {g_std:.3f}, R: {r_std:.3f}"
            )
        elif noise_std < 15.0:
            self.get_logger().warn(
                f"Moderate noise detected (gray std): {noise_std:.3f} | "
                f"B: {b_std:.3f}, G: {g_std:.3f}, R: {r_std:.3f}"
            )
        else:
            self.get_logger().error(
                f"High noise detected (gray std): {noise_std:.3f} | "
                f"B: {b_std:.3f}, G: {g_std:.3f}, R: {r_std:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PixelNoiseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
