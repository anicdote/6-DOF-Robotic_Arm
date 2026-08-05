#!/usr/bin/env python3

from __future__ import annotations

import math

import pigpio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ServoBridge(Node):

    def __init__(self):

        super().__init__("servo_bridge")

        defaults = {

            "update_rate_hz": 50.0,

            "joint_state_topic": "/joint_states",

            "joint_names": [
                "joint1_base_yaw",
                "joint2_shoulder_pitch",
                "joint3_elbow_pitch",
                "joint4_wrist_pitch",
            ],

            "gpio_pins": [
                18,
                19,
                13,
                12,
            ],

            "offsets_deg": [
                90.0,
                90.0,
                90.0,
                90.0,
            ],

            "directions": [
                1.0,
                -1.0,
                1.0,
                -1.0,
            ],

            "scales_deg_per_unit": [
                180.0 / math.pi,
                180.0 / math.pi,
                180.0 / math.pi,
                180.0 / math.pi,
            ],

            "servo_min_deg": [
                0.0,
                5.0,
                5.0,
                5.0,
            ],

            "servo_max_deg": [
                180.0,
                175.0,
                175.0,
                175.0,
            ],

            "command_min": [
                -6.2832,
                -2.2,
                -2.6,
                -2.6,
            ],

            "command_max": [
                6.2832,
                1.2,
                2.6,
                2.6,
            ],

            "max_command_rate_per_s": [
                0.8,
                0.65,
                0.7,
                0.9,
            ],
        }

        for k, v in defaults.items():
            self.declare_parameter(k, v)

        self.gpio = list(self.get_parameter("gpio_pins").value)
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.offsets = list(self.get_parameter("offsets_deg").value)
        self.directions = list(self.get_parameter("directions").value)
        self.scales = list(self.get_parameter("scales_deg_per_unit").value)
        self.servo_min = list(self.get_parameter("servo_min_deg").value)
        self.servo_max = list(self.get_parameter("servo_max_deg").value)
        self.command_min = list(self.get_parameter("command_min").value)
        self.command_max = list(self.get_parameter("command_max").value)
        self.max_rates = list(
            self.get_parameter("max_command_rate_per_s").value
        )

        self.current = [
            min(h, max(l, 0.0))
            for l, h in zip(self.command_min, self.command_max)
        ]

        self.target = list(self.current)

        self.pi = pigpio.pi()

        if not self.pi.connected:
            raise RuntimeError("Could not connect to pigpiod")

        topic = self.get_parameter("joint_state_topic").value

        self.create_subscription(
            JointState,
            topic,
            self._joint_state_callback,
            10,
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            "/arm_hardware/joint_states",
            10,
        )

        rate = float(self.get_parameter("update_rate_hz").value)

        self.timer = self.create_timer(
            1.0 / rate,
            self._update,
        )

        self.last_update = self.get_clock().now()

        self.get_logger().info("Servo Bridge Started")

    def _joint_state_callback(self, msg):

        for name, value in zip(msg.name, msg.position):

            if name not in self.joint_names:
                continue

            idx = self.joint_names.index(name)

            self.target[idx] = min(
                self.command_max[idx],
                max(self.command_min[idx], float(value)),
            )

    def _servo_angle(self, index, command):

        angle = (
            self.offsets[index]
            + self.directions[index]
            * self.scales[index]
            * command
        )

        return min(
            self.servo_max[index],
            max(self.servo_min[index], angle),
        )

    def _update(self):

        now = self.get_clock().now()

        dt = max(
            0.0,
            (now - self.last_update).nanoseconds * 1e-9,
        )

        self.last_update = now

        for i in range(len(self.target)):

            delta = self.target[i] - self.current[i]

            max_delta = self.max_rates[i] * dt

            if abs(delta) <= max_delta:
                self.current[i] = self.target[i]
            else:
                self.current[i] += math.copysign(
                    max_delta,
                    delta,
                )

            angle = self._servo_angle(
                i,
                self.current[i],
            )

            pulse = int(
                500 + angle * 2000 / 180.0
            )

            self.pi.set_servo_pulsewidth(
                self.gpio[i],
                pulse,
            )

        state = JointState()

        state.header.stamp = now.to_msg()

        state.name = self.joint_names

        state.position = self.current

        self.joint_state_pub.publish(state)


def main(args=None):

    rclpy.init(args=args)

    node = ServoBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        for pin in [18, 19, 13, 12]:
            node.pi.set_servo_pulsewidth(pin, 0)

        node.pi.stop()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()