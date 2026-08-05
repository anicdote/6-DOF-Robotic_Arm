#!/usr/bin/env python3
"""Track a selected colour and project its image centroid onto a table plane."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


HSV_RANGES = {
    "red": (
        (np.array([0, 100, 70]), np.array([10, 255, 255])),
        (np.array([170, 100, 70]), np.array([179, 255, 255])),
    ),
    "green": (
        (np.array([38, 70, 55]), np.array([85, 255, 255])),
    ),
    "blue": (
        (np.array([92, 80, 50]), np.array([132, 255, 255])),
    ),
}


class ColorTracker(Node):
    def __init__(self) -> None:
        super().__init__("color_tracker")
        self.declare_parameter("target_color", "red")
        self.declare_parameter("image_topic", "/arm_camera/image_raw")
        self.declare_parameter("camera_info_topic", "/arm_camera/camera_info")
        self.declare_parameter("fixed_frame", "base_link")
        self.declare_parameter("camera_frame_convention", "auto")
        # A contour centroid lies near the centre of a visible cube face, not
        # on its top surface. Projecting onto the face-centre plane avoids a
        # systematic range error with the low base-mounted camera.
        self.declare_parameter("object_top_z", 0.020)
        self.declare_parameter("minimum_blob_area", 350.0)
        self.declare_parameter("morphology_kernel", 5)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("target_color_topic", "/pick_place/target_color")

        self.target_color = (
            self.get_parameter("target_color").get_parameter_value().string_value
        ).lower()
        if self.target_color not in HSV_RANGES:
            raise ValueError("target_color must be red, green, or blue")

        self.fixed_frame = self.get_parameter("fixed_frame").value
        self.frame_convention = (
            self.get_parameter("camera_frame_convention").value
        ).lower()
        if self.frame_convention not in {"auto", "optical", "x_forward"}:
            raise ValueError(
                "camera_frame_convention must be auto, optical, or x_forward"
            )
        self.object_top_z = float(self.get_parameter("object_top_z").value)
        self.minimum_blob_area = float(
            self.get_parameter("minimum_blob_area").value
        )
        kernel_size = max(1, int(self.get_parameter("morphology_kernel").value))
        if kernel_size % 2 == 0:
            kernel_size += 1
        self.kernel = np.ones((kernel_size, kernel_size), np.uint8)
        self.publish_debug = bool(
            self.get_parameter("publish_debug_image").value
        )

        self.bridge = CvBridge()
        self.camera_info: CameraInfo | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, image_topic, self._image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            String,
            self.get_parameter("target_color_topic").value,
            self._target_color_callback,
            10,
        )
        self.point_pub = self.create_publisher(
            PointStamped, "/color_tracker/target_point", 10
        )
        self.debug_pub = self.create_publisher(
            Image, "/color_tracker/debug_image", 5
        )
        self.get_logger().info(
            f"Tracking {self.target_color} objects on {image_topic}"
        )

    def _target_color_callback(self, msg: String) -> None:
        color = msg.data.strip().lower()
        if color not in HSV_RANGES:
            self.get_logger().warning(f"Ignoring unsupported target colour: {color}")
            return
        if color != self.target_color:
            self.target_color = color
            self.get_logger().info(f"Dashboard selected {color}")

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.camera_info = msg

    def _mask_for_target(self, hsv: np.ndarray) -> np.ndarray:
        ranges = HSV_RANGES[self.target_color]
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

    def _pixel_to_plane(
        self, u: float, v: float, msg: Image
    ) -> PointStamped | None:
        info = self.camera_info
        if info is None:
            return None

        fx, fy = float(info.k[0]), float(info.k[4])
        cx, cy = float(info.k[2]), float(info.k[5])
        source_frame = (
            msg.header.frame_id
            or info.header.frame_id
            or "camera_optical_frame"
        )

        x_normalized = (u - cx) / fx
        y_normalized = (v - cy) / fy
        convention = self.frame_convention
        if convention == "auto":
            convention = (
                "optical" if "optical" in source_frame.lower() else "x_forward"
            )
        if convention == "optical":
            # REP-103 optical convention: +X right, +Y down, +Z forward.
            ray = np.array([x_normalized, y_normalized, 1.0])
        else:
            # Gazebo body convention: +X forward, +Y left, +Z up.
            ray = np.array([1.0, -x_normalized, -y_normalized])
        ray /= np.linalg.norm(ray)

        origin = PointStamped()
        origin.header = msg.header
        origin.header.frame_id = source_frame
        endpoint = PointStamped()
        endpoint.header = msg.header
        endpoint.header.frame_id = source_frame
        endpoint.point.x = float(ray[0])
        endpoint.point.y = float(ray[1])
        endpoint.point.z = float(ray[2])

        try:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.fixed_frame,
                    source_frame,
                    Time.from_msg(msg.header.stamp),
                )
            except TransformException:
                # Gazebo camera frames can lead joint-state TF by one update.
                # Waiting here cannot help a single-threaded executor receive
                # that update, so use the newest available transform. Target
                # localisation only runs while the arm is stationary.
                transform = self.tf_buffer.lookup_transform(
                    self.fixed_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
            world_origin = do_transform_point(origin, transform)
            world_endpoint = do_transform_point(endpoint, transform)
        except TransformException as exc:
            self.get_logger().debug(f"Camera transform unavailable: {exc}")
            return None

        dx = world_endpoint.point.x - world_origin.point.x
        dy = world_endpoint.point.y - world_origin.point.y
        dz = world_endpoint.point.z - world_origin.point.z
        if dz >= -1e-6:
            return None

        distance = (self.object_top_z - world_origin.point.z) / dz
        if distance <= 0.0 or distance > 2.0:
            return None

        target = PointStamped()
        target.header.stamp = msg.header.stamp
        target.header.frame_id = self.fixed_frame
        target.point.x = world_origin.point.x + distance * dx
        target.point.y = world_origin.point.y + distance * dy
        target.point.z = self.object_top_z
        return target

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # cv_bridge exposes several exception types
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._mask_for_target(hsv)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = [
            contour
            for contour in contours
            if cv2.contourArea(contour) >= self.minimum_blob_area
        ]

        if contours:
            contour = max(contours, key=cv2.contourArea)
            moments = cv2.moments(contour)
            if moments["m00"] > 0.0:
                u = moments["m10"] / moments["m00"]
                v = moments["m01"] / moments["m00"]
                target = self._pixel_to_plane(u, v, msg)
                if target is not None:
                    self.point_pub.publish(target)
                if self.publish_debug:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.circle(frame, (int(u), int(v)), 5, (255, 255, 255), -1)
                    label = f"{self.target_color} A={cv2.contourArea(contour):.0f}"
                    cv2.putText(
                        frame,
                        label,
                        (x, max(18, y - 7)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 255),
                        2,
                    )

        if self.publish_debug:
            debug = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            debug.header = msg.header
            self.debug_pub.publish(debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorTracker()
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
