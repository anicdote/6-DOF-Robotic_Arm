#!/usr/bin/env python3
"""Automatic colour-target pick/place sequence using lightweight analytical IK."""

from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arm_kinematics import ArmGeometry, forward_tool_position, solve_topdown_ik


ARM_JOINTS = [
    "joint1_base_yaw",
    "joint2_shoulder_pitch",
    "joint3_elbow_pitch",
    "joint4_wrist_pitch",
]

# Keep the elbow on the same positive branch used by all tabletop IK poses.
# The tool stays 109 mm above the table while the base camera observes.
OBSERVATION_POSE = [0.0, -1.0, 0.7, 1.5]


@dataclass
class MotionStep:
    name: str
    arm_positions: list[float] | None
    gripper_position: float | None
    duration: float
    grasp_command: str | None = None


class PickPlaceController(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_controller")
        defaults = {
            "detection_topic": "/color_tracker/target_point",
            "arm_command_topic": "/arm_controller/joint_trajectory",
            "gripper_command_topic": "/gripper_controller/joint_trajectory",
            "auto_start": True,
            "command_topic": "/pick_place/command",
            "target_color_topic": "/pick_place/target_color",
            "required_detections": 5,
            "pick_z": 0.025,
            "pregrasp_z": 0.090,
            "lift_z": 0.090,
            "place_x": 0.200,
            "place_y": -0.180,
            "place_z": 0.043,
            "tool_pitch": 1.250,
            "shoulder_z": 0.097,
            "shoulder_x": -0.0129,
            "shoulder_y": -0.0072,
            "upper_arm_length": 0.120,
            "forearm_to_wrist_length": 0.204,
            "wrist_to_tip_length": 0.269,
            "open_gripper_position": 0.0,
            "closed_gripper_position": 0.045,
            "gripper_joint_names": [
                "left_finger_joint",
                "right_finger_joint",
            ],
            "gripper_joint_multipliers": [-1.0, 1.0],
            "target_color": "red",
            "use_sim_grasp_assist": True,
            "sim_set_pose_service": "/world/color_pick_place/set_pose",
            "payload_tool_z_offset": -0.005,
            "detection_timeout": 4.0,
            "scan_step_duration": 2.5,
            "scan_dwell": 0.70,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.geometry = ArmGeometry(
            shoulder_z=float(self.get_parameter("shoulder_z").value),
            shoulder_x=float(self.get_parameter("shoulder_x").value),
            shoulder_y=float(self.get_parameter("shoulder_y").value),
            upper_arm=float(self.get_parameter("upper_arm_length").value),
            forearm_to_wrist=float(
                self.get_parameter("forearm_to_wrist_length").value
            ),
            wrist_to_tip=float(
                self.get_parameter("wrist_to_tip_length").value
            ),
        )
        self.tool_pitch = float(self.get_parameter("tool_pitch").value)
        self.open_position = float(
            self.get_parameter("open_gripper_position").value
        )
        self.closed_position = float(
            self.get_parameter("closed_gripper_position").value
        )
        self.gripper_joint_names = list(
            self.get_parameter("gripper_joint_names").value
        )
        self.gripper_joint_multipliers = list(
            self.get_parameter("gripper_joint_multipliers").value
        )
        if len(self.gripper_joint_names) != len(self.gripper_joint_multipliers):
            raise ValueError(
                "gripper_joint_names and gripper_joint_multipliers must match"
            )
        self.target_color = str(self.get_parameter("target_color").value).lower()
        if self.target_color not in {"red", "green", "blue"}:
            raise ValueError("target_color must be red, green, or blue")
        self.use_sim_grasp_assist = bool(
            self.get_parameter("use_sim_grasp_assist").value
        )
        self.payload_tool_z_offset = float(
            self.get_parameter("payload_tool_z_offset").value
        )
        self.detection_timeout = float(
            self.get_parameter("detection_timeout").value
        )
        self.scan_step_duration = float(
            self.get_parameter("scan_step_duration").value
        )
        self.scan_dwell = float(self.get_parameter("scan_dwell").value)
        self.required_detections = int(
            self.get_parameter("required_detections").value
        )
        self.auto_start = bool(self.get_parameter("auto_start").value)

        self.arm_pub = self.create_publisher(
            JointTrajectory, self.get_parameter("arm_command_topic").value, 10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            self.get_parameter("gripper_command_topic").value,
            10,
        )
        self.status_pub = self.create_publisher(
            String, "/pick_place/status", 10
        )
        self.pose_client = self.create_client(
            SetEntityPose,
            self.get_parameter("sim_set_pose_service").value,
        )
        self.create_subscription(
            PointStamped,
            self.get_parameter("detection_topic").value,
            self._detection_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("command_topic").value,
            self._command_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("target_color_topic").value,
            self._target_color_callback,
            10,
        )
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        # The dashboard never talks to a hardware controller directly.  It
        # sends manual requests here so autonomous motion can be cancelled
        # before a human takes over.
        self.create_subscription(
            JointTrajectory,
            "/pick_place/manual_arm",
            self._manual_arm_callback,
            10,
        )
        self.create_subscription(
            JointTrajectory,
            "/pick_place/manual_gripper",
            self._manual_gripper_callback,
            10,
        )

        self.detections: deque[tuple[float, float]] = deque(
            maxlen=max(3, self.required_detections)
        )
        self.steps: deque[MotionStep] = deque()
        self.enabled = self.auto_start
        self.busy = False
        self.task_complete = False
        self.scanning = False
        self.search_exhausted = False
        self.scan_targets: deque[float] = deque()
        self.next_step_time = self.get_clock().now()
        self.detection_resume_time = self.get_clock().now()
        self.search_deadline = self.get_clock().now()
        self.scan_next_time = self.get_clock().now()
        self.startup_sent = False
        self.carrying = False
        self.latest_arm_positions: list[float] | None = None
        self.pose_request_pending = None
        self.pose_service_warning_sent = False
        # A 50 Hz payload update prevents the carried Gazebo model from
        # visibly stepping between poses while the arm trajectory is moving.
        self.timer = self.create_timer(0.02, self._timer_callback)
        self.get_logger().info(
            "Pick/place controller ready; use the dashboard or START command"
        )

    @staticmethod
    def _duration_msg(seconds: float) -> Duration:
        whole = int(seconds)
        return Duration(sec=whole, nanosec=int((seconds - whole) * 1e9))

    def _publish_arm(self, positions: list[float], duration: float) -> None:
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = self._duration_msg(duration)
        msg.points = [point]
        self.arm_pub.publish(msg)

    def _publish_gripper(self, position: float, duration: float) -> None:
        msg = JointTrajectory()
        msg.joint_names = self.gripper_joint_names
        point = JointTrajectoryPoint()
        point.positions = [
            multiplier * position
            for multiplier in self.gripper_joint_multipliers
        ]
        point.time_from_start = self._duration_msg(duration)
        msg.points = [point]
        self.gripper_pub.publish(msg)

    def _publish_status(self, status: str) -> None:
        self.status_pub.publish(String(data=status))

    def _reset_cycle_state(self) -> None:
        self.detections.clear()
        self.steps.clear()
        self.busy = False
        self.task_complete = False
        self.scanning = False
        self.search_exhausted = False
        self.scan_targets.clear()
        self.startup_sent = False

    def _command_callback(self, msg: String) -> None:
        command = msg.data.strip().upper()
        if command == "START":
            if self.busy:
                self.get_logger().warning("START ignored: sequence is already moving")
                return
            if self.carrying:
                self._publish_grasp_command("detach")
            self._reset_cycle_state()
            self.enabled = True
            self._publish_status("STARTING")
            self.get_logger().info(
                f"Dashboard started autonomous {self.target_color} pick/place"
            )
            return

        if command == "STOP":
            self.enabled = False
            self.steps.clear()
            self.busy = False
            self.scanning = False
            self.scan_targets.clear()
            if self.latest_arm_positions is not None:
                self._publish_arm(self.latest_arm_positions, 0.20)
            self._publish_status("STOPPED")
            self.get_logger().warning("Dashboard stop: holding current arm pose")
            return

        if command == "RESET":
            self.enabled = False
            if self.carrying:
                self._publish_grasp_command("detach")
            self._reset_cycle_state()
            self.carrying = False
            self._publish_gripper(self.open_position, 0.75)
            self._publish_arm(OBSERVATION_POSE, 2.0)
            self._reset_sim_target()
            self._publish_status("READY")
            self.get_logger().info(
                "Dashboard reset: arm and target returned to their start poses"
            )
            return

        self.get_logger().warning(f"Unknown dashboard command: {command}")

    def _target_color_callback(self, msg: String) -> None:
        color = msg.data.strip().lower()
        if color not in {"red", "green", "blue"}:
            self.get_logger().warning(f"Ignoring unsupported target colour: {color}")
            return
        if self.busy or self.carrying:
            self.get_logger().warning("Target colour cannot change during motion")
            return
        self.target_color = color
        self.detections.clear()
        self.task_complete = False
        self.search_exhausted = False
        self._publish_status(f"TARGET_COLOR:{color.upper()}")

    def _joint_state_callback(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in ARM_JOINTS):
            self.latest_arm_positions = [positions[name] for name in ARM_JOINTS]

    def _take_manual_control(self) -> None:
        """Stop an autonomous cycle and make the controller safe to jog."""
        if self.carrying:
            self._publish_grasp_command("detach")
        self.enabled = False
        self.busy = False
        self.scanning = False
        self.steps.clear()
        self.scan_targets.clear()
        self._publish_status("MANUAL_CONTROL")

    def _manual_arm_callback(self, msg: JointTrajectory) -> None:
        if msg.joint_names != ARM_JOINTS or len(msg.points) != 1:
            self.get_logger().warning("Rejected invalid manual arm trajectory")
            return
        positions = list(msg.points[0].positions)
        if len(positions) != len(ARM_JOINTS) or not all(
            math.isfinite(value) for value in positions
        ):
            self.get_logger().warning("Rejected non-finite manual arm command")
            return
        self._take_manual_control()
        seconds = max(0.25, msg.points[0].time_from_start.sec +
                      msg.points[0].time_from_start.nanosec * 1e-9)
        self._publish_arm(positions, seconds)

    def _manual_gripper_callback(self, msg: JointTrajectory) -> None:
        dashboard_joints = ["left_finger_joint", "right_finger_joint"]
        if (
            msg.joint_names not in (self.gripper_joint_names, dashboard_joints)
            or len(msg.points) != 1
        ):
            self.get_logger().warning("Rejected invalid manual gripper trajectory")
            return
        positions = list(msg.points[0].positions)
        if len(positions) != len(msg.joint_names) or not all(
            math.isfinite(value) for value in positions
        ):
            self.get_logger().warning("Rejected non-finite manual gripper command")
            return
        self._take_manual_control()
        # The dashboard always uses the two-jaw simulation convention
        # [-opening, opening].  Hardware may instead configure one physical
        # servo, so derive the same positive opening from either format.
        if msg.joint_names == dashboard_joints:
            opening = positions[1]
        else:
            multiplier = self.gripper_joint_multipliers[0]
            if abs(multiplier) < 1e-9:
                self.get_logger().warning("Rejected manual gripper: zero multiplier")
                return
            opening = positions[0] / multiplier
        if not math.isfinite(opening) or opening < 0.0:
            self.get_logger().warning("Rejected invalid manual gripper opening")
            return
        seconds = max(0.25, msg.points[0].time_from_start.sec +
                      msg.points[0].time_from_start.nanosec * 1e-9)
        self._publish_gripper(opening, seconds)

    def _update_sim_payload(self) -> None:
        if (
            not self.use_sim_grasp_assist
            or not self.carrying
            or self.latest_arm_positions is None
        ):
            return
        if (
            self.pose_request_pending is not None
            and not self.pose_request_pending.done()
        ):
            return
        if not self.pose_client.service_is_ready():
            if not self.pose_service_warning_sent:
                self.pose_service_warning_sent = True
                self.get_logger().warning(
                    "Gazebo payload pose service is not ready; waiting"
                )
            return

        x, y, z = forward_tool_position(self.latest_arm_positions, self.geometry)
        self._set_sim_target_pose(x, y, max(0.020, z + self.payload_tool_z_offset))

    def _set_sim_target_pose(self, x: float, y: float, z: float) -> None:
        """Move the non-physical grasp-assist cube without fighting Gazebo."""
        if self.pose_request_pending is not None and not self.pose_request_pending.done():
            return
        if not self.pose_client.service_is_ready():
            if not self.pose_service_warning_sent:
                self.pose_service_warning_sent = True
                self.get_logger().warning(
                    "Gazebo payload pose service is not ready; waiting"
                )
            return
        request = SetEntityPose.Request()
        request.entity.name = f"{self.target_color}_pick_cube"
        request.entity.type = 2  # ros_gz_interfaces/Entity.MODEL
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z
        request.pose.orientation.w = 1.0
        self.pose_request_pending = self.pose_client.call_async(request)

    def _reset_sim_target(self) -> None:
        """Reset the selected cube too, so Reset always gives a repeatable run."""
        starts = {
            "red": (0.260, 0.020, 0.020),
            "green": (0.220, 0.145, 0.020),
            "blue": (0.210, -0.105, 0.020),
        }
        if self.use_sim_grasp_assist:
            self._set_sim_target_pose(*starts[self.target_color])

    def _publish_grasp_command(self, command: str) -> None:
        if not self.use_sim_grasp_assist:
            return
        if command == "attach":
            self.carrying = True
            self._update_sim_payload()
        elif command == "detach":
            self._update_sim_payload()
            self.carrying = False
        else:
            raise ValueError(f"Unknown grasp command: {command}")

    def _startup_pose(self) -> None:
        self._publish_gripper(self.open_position, 1.0)
        self._publish_arm(OBSERVATION_POSE, 3.0)
        self._publish_status("SEARCHING")
        self.get_logger().info("Moved to camera observation pose")

    def _start_scan(self, now) -> None:
        self.scanning = True
        self.detections.clear()
        self.scan_targets = deque(
            [0.5 * math.pi, math.pi, 1.5 * math.pi, 2.0 * math.pi]
        )
        self.scan_next_time = now
        self._publish_status("SCANNING_360")
        self.get_logger().warning(
            "Target not visible; starting one 360-degree base scan"
        )

    def _detection_callback(self, msg: PointStamped) -> None:
        now = self.get_clock().now()
        if (
            not self.enabled
            or not self.startup_sent
            or now < self.detection_resume_time
            or self.busy
            or self.task_complete
            or self.search_exhausted
            or msg.header.frame_id != "base_link"
        ):
            return
        if not (0.10 <= msg.point.x <= 0.38 and abs(msg.point.y) <= 0.25):
            self.get_logger().warning(
                f"Ignoring target outside safe tabletop area: "
                f"({msg.point.x:.3f}, {msg.point.y:.3f})"
            )
            return
        self.detections.append((msg.point.x, msg.point.y))
        if len(self.detections) < self.required_detections:
            return

        x = statistics.median(p[0] for p in self.detections)
        y = statistics.median(p[1] for p in self.detections)
        self.detections.clear()
        try:
            self._build_sequence(x, y)
        except ValueError as exc:
            self.get_logger().error(
                f"Cannot plan pick at ({x:.3f}, {y:.3f}): {exc}"
            )

    def _ik(self, x: float, y: float, z: float) -> list[float]:
        return solve_topdown_ik(
            x,
            y,
            z,
            tool_pitch=self.tool_pitch,
            geometry=self.geometry,
        )

    def _build_sequence(self, x: float, y: float) -> None:
        pick_z = float(self.get_parameter("pick_z").value)
        pregrasp_z = float(self.get_parameter("pregrasp_z").value)
        lift_z = float(self.get_parameter("lift_z").value)
        place_x = float(self.get_parameter("place_x").value)
        place_y = float(self.get_parameter("place_y").value)
        place_z = float(self.get_parameter("place_z").value)

        pick_pre = self._ik(x, y, pregrasp_z)
        pick = self._ik(x, y, pick_z)
        pick_lift = self._ik(x, y, lift_z)
        place_pre = self._ik(place_x, place_y, lift_z)
        place = self._ik(place_x, place_y, place_z)

        self.steps = deque(
            [
                MotionStep("open gripper", None, self.open_position, 1.0),
                MotionStep("approach target", pick_pre, None, 3.0),
                MotionStep("descend", pick, None, 2.0),
                MotionStep("secure grasp", None, None, 0.35, "attach"),
                MotionStep("close gripper", None, self.closed_position, 1.2),
                MotionStep("lift object", pick_lift, None, 3.0),
                MotionStep("move over tray", place_pre, None, 4.0),
                MotionStep("lower into tray", place, None, 2.0),
                MotionStep("release grasp", None, None, 0.25, "detach"),
                MotionStep("release object", None, self.open_position, 1.0),
                MotionStep("retreat", place_pre, None, 2.0),
                MotionStep(
                    "return to observation",
                    OBSERVATION_POSE,
                    None,
                    3.0,
                ),
            ]
        )
        self.scanning = False
        self.scan_targets.clear()
        self.busy = True
        self.next_step_time = self.get_clock().now()
        self._publish_status("TARGET_FOUND")
        self.get_logger().info(
            f"Accepted target at x={x:.3f}, y={y:.3f}; starting sequence"
        )

    def _timer_callback(self) -> None:
        now = self.get_clock().now()
        self._update_sim_payload()
        if not self.enabled:
            return
        if not self.startup_sent:
            self.startup_sent = True
            self._startup_pose()
            self.next_step_time = now + RclpyDuration(seconds=3.2)
            self.detection_resume_time = self.next_step_time
            self.search_deadline = self.next_step_time + RclpyDuration(
                seconds=self.detection_timeout
            )
            return

        if self.busy:
            if now < self.next_step_time:
                return
            if not self.steps:
                self.busy = False
                self.task_complete = True
                self.enabled = False
                self._publish_status("COMPLETE")
                self.get_logger().info("Pick/place sequence complete")
                return

            step = self.steps.popleft()
            if step.arm_positions is not None:
                self._publish_arm(step.arm_positions, step.duration)
            if step.gripper_position is not None:
                self._publish_gripper(step.gripper_position, step.duration)
            if step.grasp_command is not None:
                self._publish_grasp_command(step.grasp_command)
            self._publish_status(f"MOVING: {step.name}")
            self.get_logger().info(f"Step: {step.name}")
            self.next_step_time = now + RclpyDuration(
                seconds=step.duration + 0.25
            )
            return

        if self.scanning:
            if now < self.scan_next_time:
                return
            if self.scan_targets:
                yaw = self.scan_targets.popleft()
                pose = list(OBSERVATION_POSE)
                pose[0] = yaw
                self._publish_arm(pose, self.scan_step_duration)
                self.detections.clear()
                self.detection_resume_time = now + RclpyDuration(
                    seconds=self.scan_step_duration
                )
                self.scan_next_time = self.detection_resume_time + RclpyDuration(
                    seconds=self.scan_dwell
                )
                self.get_logger().info(
                    f"Scanning at base yaw {math.degrees(yaw):.0f} degrees"
                )
                return

            self.scanning = False
            self.search_exhausted = True
            self._publish_status("OBJECT_NOT_FOUND")
            self.get_logger().error(
                "OBJECT NOT FOUND after one complete 360-degree scan; stopping"
            )
            return

        if (
            not self.task_complete
            and not self.search_exhausted
            and now >= self.search_deadline
        ):
            self._start_scan(now)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickPlaceController()
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
