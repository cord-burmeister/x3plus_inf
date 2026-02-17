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
        # Output convention for roll/pitch: 'z_forward' (optical) or 'x_forward'
        #self.declare_parameter('output_convention', 'z_forward')
        self.declare_parameter('output_convention', 'x_forward')

        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_prefix = self.get_parameter('tag_prefix').value
        self.tag_ids = self.get_parameter('tag_ids').value
        self.output_convention = self.get_parameter('output_convention').value

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

        # Tag positions in plane frame (z should be ~0)
        tag_positions_h = np.hstack([tag_positions, np.ones((tag_positions.shape[0], 1))])
        tag_positions_plane = (T_cam_plane @ tag_positions_h.T).T
        z_vals = tag_positions_plane[:, 2]
        z_mean = float(np.mean(z_vals))
        z_mean_abs = float(np.mean(np.abs(z_vals)))
        z_rmse = float(np.sqrt(np.mean(z_vals ** 2)))

        cam_pos = T_cam_plane[:3, 3]
        height = abs(cam_pos[2])

        # Roll/pitch from plane normal
        # If output_convention == 'z_forward', assume camera frame is optical: x right, y down, z forward
        # If output_convention == 'x_forward', convert to: x forward, y left, z up
        if self.output_convention == 'x_forward':
            # optical (x right, y down, z forward) -> x_forward (x forward, y left, z up)
            nx, ny, nz = plane_normal
            nx, ny, nz = nz, -nx, -ny
        else:
            nx, ny, nz = plane_normal

        roll = np.arctan2(ny, nz)
        pitch = np.arctan2(-nx, np.sqrt(ny * ny + nz * nz))
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)

        self.get_logger().info(
            f"Height: {height:.3f} m | Roll: {roll_deg:.2f} deg | Pitch: {pitch_deg:.2f} deg | "
            f"Z-err mean: {z_mean:.4f} m | Z-err mean|abs|: {z_mean_abs:.4f} m | Z-err RMSE: {z_rmse:.4f} m | "
            f"Normal: {plane_normal}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraPlaneCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
