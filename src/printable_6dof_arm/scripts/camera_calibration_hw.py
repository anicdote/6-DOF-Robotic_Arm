#!/usr/bin/env python3
"""Undistort hardware camera images using an optional OpenCV calibration."""

from __future__ import annotations

import copy
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraCalibrationHardware(Node):
    def __init__(self) -> None:
        super().__init__("camera_calibration_hw")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("rectified_image_topic", "/hardware_camera/image_rect")
        self.declare_parameter("rectified_camera_info_topic", "/hardware_camera/camera_info")

        self.bridge = CvBridge()
        self.camera_matrix: np.ndarray | None = None
        self.distortion: np.ndarray | None = None
        self.latest_camera_info: CameraInfo | None = None
        self._load_calibration(self.get_parameter("calibration_file").value)

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
        self.image_pub = self.create_publisher(
            Image, self.get_parameter("rectified_image_topic").value, 5
        )
        self.info_pub = self.create_publisher(
            CameraInfo,
            self.get_parameter("rectified_camera_info_topic").value,
            5,
        )
        self.get_logger().info(f"Calibrating hardware images from {image_topic}")

    def _load_calibration(self, calibration_file: str) -> None:
        if not calibration_file:
            self.get_logger().warning(
                "No calibration file configured; forwarding raw camera images"
            )
            return

        path = Path(calibration_file)
        if not path.is_file():
            self.get_logger().warning(
                f"Calibration file unavailable: {path}; forwarding raw images"
            )
            return

        storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        if not storage.isOpened():
            self.get_logger().warning(
                f"Could not read calibration file: {path}; forwarding raw images"
            )
            return
        try:
            self.camera_matrix = self._read_matrix(
                storage, ("camera_matrix", "K")
            )
            self.distortion = self._read_matrix(
                storage, ("distortion_coefficients", "dist_coeffs", "D")
            )
        finally:
            storage.release()

        if (
            self.camera_matrix is None
            or self.camera_matrix.shape != (3, 3)
            or self.distortion is None
            or self.distortion.size < 4
        ):
            self.camera_matrix = None
            self.distortion = None
            self.get_logger().warning(
                f"Calibration file is missing camera intrinsics: {path}; "
                "forwarding raw images"
            )
            return
        self.distortion = self.distortion.reshape(-1, 1)
        self.get_logger().info(f"Loaded camera calibration from {path}")

    @staticmethod
    def _read_matrix(
        storage: cv2.FileStorage, names: tuple[str, ...]
    ) -> np.ndarray | None:
        for name in names:
            matrix = storage.getNode(name).mat()
            if matrix is not None and matrix.size:
                return matrix.astype(np.float64)
        return None

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def _rectified_camera_info(self, msg: Image) -> CameraInfo:
        info = CameraInfo()
        if self.latest_camera_info is not None:
            info = copy.deepcopy(self.latest_camera_info)
        info.header = msg.header
        info.width = msg.width
        info.height = msg.height
        if self.camera_matrix is not None:
            info.k = self.camera_matrix.reshape(-1).tolist()
            info.d = [0.0] * len(info.d or self.distortion.reshape(-1))
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [
                self.camera_matrix[0, 0], 0.0, self.camera_matrix[0, 2], 0.0,
                0.0, self.camera_matrix[1, 1], self.camera_matrix[1, 2], 0.0,
                0.0, 0.0, 1.0, 0.0,
            ]
        return info

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # cv_bridge exposes several exception types
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        if self.camera_matrix is not None and self.distortion is not None:
            frame = cv2.undistort(frame, self.camera_matrix, self.distortion)
        output = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        output.header = msg.header
        self.image_pub.publish(output)
        self.info_pub.publish(self._rectified_camera_info(msg))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraCalibrationHardware()
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
