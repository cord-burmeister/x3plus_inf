#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

class CameraPlaneCalibrator(Node):
    def __init__(self):
        super().__init__('camera_plane_calibrator')

        self.declare_parameter('camera_frame', 'c920_camera_link')
        self.declare_parameter('tag_prefix', 'tag36h11')
        self.declare_parameter('tag_ids', [0,1,2,3,4,5,6,7,8,9])

        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_prefix = self.get_parameter('tag_prefix').value
        self.tag_ids = self.get_parameter('tag_ids').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.2, self.compute_pose)

        self.get_logger().info("Camera-plane calibration node started")

    # -----------------------------
    # Plane fitting
    # -----------------------------
    def fit_plane(self, points):
        centroid = np.mean(points, axis=0)
        _, _, vh = np.linalg.svd(points - centroid)
        normal = vh[-1]
        return centroid, normal / np.linalg.norm(normal)

    # -----------------------------
    # Build plane coordinate frame
    # -----------------------------
    def plane_frame(self, point, normal):
        z = normal
        tmp = np.array([1,0,0]) if abs(z[0]) < 0.9 else np.array([0,1,0])
        x = np.cross(tmp, z)
        x /= np.linalg.norm(x)
        y = np.cross(z, x)

        T = np.eye(4)
        T[:3, :3] = np.vstack([x, y, z]).T
        T[:3, 3] = point
        return T

    # -----------------------------
    # Invert transform
    # -----------------------------
    def invert(self, T):
        Rm = T[:3, :3]
        t = T[:3, 3]
        Tinv = np.eye(4)
        Tinv[:3, :3] = Rm.T
        Tinv[:3, 3] = -Rm.T @ t
        return Tinv

    # -----------------------------
    # Main computation
    # -----------------------------
    def compute_pose(self):
        tag_positions = []

        for tag_id in self.tag_ids:
            tag_frame = f"{self.tag_prefix}:{tag_id}"

            try:
                tf = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    tag_frame,
                    rclpy.time.Time()
                )

                p = np.array([
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z
                ])

                tag_positions.append(p)

            except Exception:
                continue

        if len(tag_positions) < 3:
            self.get_logger().warn("Not enough tags visible to fit plane")
            return

        tag_positions = np.array(tag_positions)

        # Fit plane
        plane_point, plane_normal = self.fit_plane(tag_positions)

        # Build plane frame
        T_plane = self.plane_frame(plane_point, plane_normal)

        # Camera pose in plane frame
        T_cam_plane = self.invert(T_plane)

        cam_pos = T_cam_plane[:3, 3]
        height = abs(cam_pos[2])

        R_cp = T_cam_plane[:3, :3]
        pitch = np.arctan2(-R_cp[2, 0], R_cp[2, 2])
        pitch_deg = np.degrees(pitch)

        pitch_rad = np.pi/2 - pitch

        self.get_logger().info(
            f"Height: {height:.3f} m | Pitch: {pitch_rad:.2f} rad | Normal: {plane_normal}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraPlaneCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
