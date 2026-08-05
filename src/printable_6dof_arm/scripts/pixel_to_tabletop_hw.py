#!/usr/bin/env python3
"""Project calibrated image pixels onto a configurable robot-base table plane."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the camera-to-base rotation for fixed-frame roll, pitch, yaw."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


class PixelToTabletopHardware(Node):
    def __init__(self) -> None:
        super().__init__("pixel_to_tabletop_hw")
        self.declare_parameter("pixel_topic", "/color_tracker/target_pixel")
        self.declare_parameter("camera_info_topic", "/hardware_camera/camera_info")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tabletop_z", 0.0)
        self.declare_parameter("camera_x", 0.0)
        self.declare_parameter("camera_y", 0.0)
        self.declare_parameter("camera_z", 0.30)
        self.declare_parameter("camera_roll", 0.0)
        self.declare_parameter("camera_pitch", 0.0)
        self.declare_parameter("camera_yaw", 0.0)

        self.camera_info: CameraInfo | None = None
        self.camera_origin = np.array(
            [
                float(self.get_parameter("camera_x").value),
                float(self.get_parameter("camera_y").value),
                float(self.get_parameter("camera_z").value),
            ]
        )
        self.rotation = rotation_from_rpy(
            float(self.get_parameter("camera_roll").value),
            float(self.get_parameter("camera_pitch").value),
            float(self.get_parameter("camera_yaw").value),
        )
        self.tabletop_z = float(self.get_parameter("tabletop_z").value)
        self.base_frame = self.get_parameter("base_frame").value

        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Point, self.get_parameter("pixel_topic").value, self._pixel_callback, 10
        )
        self.point_pub = self.create_publisher(
            PointStamped, "/hardware_vision/target_point", 10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, "/hardware_vision/target_pose", 10
        )
        self.detected_pub = self.create_publisher(
            Bool, "/hardware_vision/projected", 10
        )
        self.get_logger().info(
            "Projecting camera rays onto the tabletop using configured extrinsics"
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.camera_info = msg

    def _pixel_callback(self, msg: Point) -> None:
        info = self.camera_info
        if info is None:
            self.get_logger().warning("Waiting for calibrated camera intrinsics")
            return

        fx, fy = float(info.k[0]), float(info.k[4])
        cx, cy = float(info.k[2]), float(info.k[5])
        ray_camera = np.array([(msg.x - cx) / fx, (msg.y - cy) / fy, 1.0])
        ray_camera /= np.linalg.norm(ray_camera)
        ray_base = self.rotation @ ray_camera
        if ray_base[2] >= -1e-6:
            self.get_logger().warning("Camera ray does not intersect the tabletop")
            return

        distance = (self.tabletop_z - self.camera_origin[2]) / ray_base[2]
        if distance <= 0.0:
            self.get_logger().warning("Tabletop plane is behind the camera")
            return
        target = self.camera_origin + distance * ray_base

        header = self.get_clock().now().to_msg()
        point = PointStamped()
        point.header.stamp = header
        point.header.frame_id = self.base_frame
        point.point.x = float(target[0])
        point.point.y = float(target[1])
        point.point.z = float(target[2])
        self.point_pub.publish(point)

        pose = PoseStamped()
        pose.header = point.header
        pose.pose.position = point.point
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        status = Bool()
        status.data = True
        self.detected_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PixelToTabletopHardware()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
