#!/usr/bin/env python3
"""Track a selected colour and publish its image centroid for camera tests."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String


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


class ColorTrackerHardwareTest(Node):
    def __init__(self) -> None:
        super().__init__("color_tracker_hw_test")
        self.declare_parameter("target_color", "red")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("minimum_blob_area", 350.0)
        self.declare_parameter("morphology_kernel", 5)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("target_color_topic", "/pick_place/target_color")

        self.target_color = (
            self.get_parameter("target_color").get_parameter_value().string_value
        ).lower()
        if self.target_color not in HSV_RANGES:
            raise ValueError("target_color must be red, green, or blue")

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
            Point, "/color_tracker/target_pixel", 10
        )
        self.centroid_pub = self.create_publisher(
            Point, "/color_tracker/centroid", 10
        )
        self.area_pub = self.create_publisher(
            Float32, "/color_tracker/contour_area", 10
        )
        self.detected_pub = self.create_publisher(
            Bool, "/color_tracker/detected", 10
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

        detected = False
        if contours:
            contour = max(contours, key=cv2.contourArea)
            moments = cv2.moments(contour)
            if moments["m00"] > 0.0:
                u = moments["m10"] / moments["m00"]
                v = moments["m01"] / moments["m00"]
                target = Point()
                target.x = float(u)
                target.y = float(v)
                target.z = float(cv2.contourArea(contour))
                self.point_pub.publish(target)
                centroid = Point()
                centroid.x = float(u)
                centroid.y = float(v)
                self.centroid_pub.publish(centroid)
                area = Float32()
                area.data = float(cv2.contourArea(contour))
                self.area_pub.publish(area)
                detected = True
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

        status = Bool()
        status.data = detected
        self.detected_pub.publish(status)

        if self.publish_debug:
            debug = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            debug.header = msg.header
            self.debug_pub.publish(debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorTrackerHardwareTest()
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
