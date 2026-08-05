#!/usr/bin/env python3
"""Small deterministic kinematics library for tabletop operation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence


ARM_JOINT_LIMITS = (
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.20, 1.20),
    (-2.60, 2.60),
    (-2.60, 2.60),
)


@dataclass(frozen=True)
class ArmGeometry:
    """Dimensions measured from the normalized user CAD meshes."""

    shoulder_z: float = 0.097
    shoulder_x: float = -0.0129
    shoulder_y: float = -0.0072
    upper_arm: float = 0.120
    forearm_to_wrist: float = 0.204
    wrist_to_tip: float = 0.269


def within_limits(
    joints: Sequence[float],
    limits: Sequence[tuple[float, float]] = ARM_JOINT_LIMITS,
) -> bool:
    return len(joints) == len(limits) and all(
        low - 1e-9 <= value <= high + 1e-9
        for value, (low, high) in zip(joints, limits)
    )


def forward_tool_position(
    joints: Sequence[float], geometry: ArmGeometry = ArmGeometry()
) -> tuple[float, float, float]:
    """Return tool position for the CAD yaw/pitch/pitch/pitch chain."""
    if len(joints) != 4:
        raise ValueError("Expected four arm joint positions")

    q1, q2, q3, q4 = joints
    elbow_angle = q2 + q3
    tool_angle = elbow_angle + q4
    radial = (
        geometry.upper_arm * math.cos(q2)
        + geometry.forearm_to_wrist * math.cos(elbow_angle)
        + geometry.wrist_to_tip * math.cos(tool_angle)
    )
    z = (
        geometry.shoulder_z
        - geometry.upper_arm * math.sin(q2)
        - geometry.forearm_to_wrist * math.sin(elbow_angle)
        - geometry.wrist_to_tip * math.sin(tool_angle)
    )
    local_x = geometry.shoulder_x + radial
    local_y = geometry.shoulder_y
    return (
        local_x * math.cos(q1) - local_y * math.sin(q1),
        local_x * math.sin(q1) + local_y * math.cos(q1),
        z,
    )


def solve_topdown_ik(
    x: float,
    y: float,
    z: float,
    tool_pitch: float = 1.250,
    geometry: ArmGeometry = ArmGeometry(),
    limits: Sequence[tuple[float, float]] = ARM_JOINT_LIMITS,
) -> list[float]:
    """Solve a downward tabletop pose for the CAD arm.

    `tool_pitch` is measured about +Y from horizontal. Positive pitch points
    the tool toward the table. The positive-elbow branch is preferred so a
    vertical approach cannot jump to the opposite configuration midway.
    """
    target_radius = math.hypot(x, y)
    if target_radius <= abs(geometry.shoulder_y):
        raise ValueError("Target is too close to the offset shoulder axis")
    target_angle = math.atan2(y, x)
    local_target_y_angle = math.asin(geometry.shoulder_y / target_radius)
    q1 = target_angle - local_target_y_angle
    radial_target = (
        math.sqrt(target_radius * target_radius - geometry.shoulder_y**2)
        - geometry.shoulder_x
    )

    wrist_r = radial_target - geometry.wrist_to_tip * math.cos(tool_pitch)
    wrist_z = z + geometry.wrist_to_tip * math.sin(tool_pitch)
    vertical_down = -(wrist_z - geometry.shoulder_z)

    l1 = geometry.upper_arm
    l2 = geometry.forearm_to_wrist
    cosine_q3 = (
        wrist_r * wrist_r
        + vertical_down * vertical_down
        - l1 * l1
        - l2 * l2
    ) / (2.0 * l1 * l2)

    if cosine_q3 < -1.000001 or cosine_q3 > 1.000001:
        raise ValueError(
            f"Target ({x:.3f}, {y:.3f}, {z:.3f}) is outside the arm workspace"
        )
    cosine_q3 = min(1.0, max(-1.0, cosine_q3))

    candidates: list[list[float]] = []
    elbow_magnitude = math.acos(cosine_q3)
    for q3 in (elbow_magnitude, -elbow_magnitude):
        q2 = math.atan2(vertical_down, wrist_r) - math.atan2(
            l2 * math.sin(q3), l1 + l2 * math.cos(q3)
        )
        q4 = tool_pitch - q2 - q3
        solution = [q1, q2, q3, q4]
        if within_limits(solution, limits):
            candidates.append(solution)

    if not candidates:
        raise ValueError(
            "A geometric solution exists, but it violates one or more servo limits"
        )

    return max(candidates, key=lambda q: q[2])


def position_error(
    expected: Iterable[float], actual: Iterable[float]
) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(expected, actual)))


if __name__ == "__main__":
    sample = (0.260, 0.020, 0.040)
    q = solve_topdown_ik(*sample)
    reached = forward_tool_position(q)
    print("joints_rad:", " ".join(f"{value:.5f}" for value in q))
    print("tool_xyz_m:", " ".join(f"{value:.5f}" for value in reached))
    print("position_error_m:", f"{position_error(sample, reached):.8f}")
