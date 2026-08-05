#!/usr/bin/env python3
"""Offline structural, kinematic, and print-mesh validation."""

from __future__ import annotations

import collections
import math
import py_compile
import re
import struct
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / "scripts"
sys.path.insert(0, str(SCRIPTS))

from arm_kinematics import (  # noqa: E402
    ARM_JOINT_LIMITS,
    ArmGeometry,
    forward_tool_position,
    position_error,
    solve_topdown_ik,
    within_limits,
)


ARM_JOINTS = [
    "joint1_base_yaw",
    "joint2_shoulder_pitch",
    "joint3_elbow_pitch",
    "joint4_wrist_pitch",
]


def parse(path: Path) -> ET.Element:
    return ET.parse(path).getroot()


def stl_vertices(path: Path) -> list[tuple[float, float, float]]:
    data = path.read_bytes()
    if len(data) >= 84:
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        if 84 + 50 * triangle_count == len(data):
            vertices: list[tuple[float, float, float]] = []
            for index in range(triangle_count):
                values = struct.unpack_from("<12fH", data, 84 + index * 50)
                vertices.extend(
                    tuple(values[offset : offset + 3])
                    for offset in (3, 6, 9)
                )
            return vertices

    pattern = re.compile(
        r"^\s*vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)",
        re.MULTILINE,
    )
    return [
        tuple(float(value) for value in match.groups())
        for match in pattern.finditer(data.decode("utf-8"))
    ]


class XmlAndPackageTests(unittest.TestCase):
    def test_all_xml_files_parse(self) -> None:
        files = [
            PACKAGE / "package.xml",
            PACKAGE / "urdf" / "printable_6dof_arm.urdf.xacro",
            PACKAGE / "urdf" / "printable_6dof_arm.urdf",
            PACKAGE / "worlds" / "color_pick_place.sdf",
        ]
        for file in files:
            with self.subTest(file=file.name):
                parse(file)

    def test_python_files_compile(self) -> None:
        files = list((PACKAGE / "scripts").glob("*.py"))
        files += list((PACKAGE / "launch").glob("*.py"))
        for file in files:
            with self.subTest(file=file.name):
                py_compile.compile(str(file), doraise=True)

    def test_pick_place_controller_imports_from_install_directory(self) -> None:
        import pick_place_controller

        self.assertTrue(callable(pick_place_controller.solve_topdown_ik))

    def test_color_tracker_has_tf_timestamp_fallback(self) -> None:
        source = (SCRIPTS / "color_tracker.py").read_text(encoding="utf-8")
        self.assertIn("Time.from_msg(msg.header.stamp)", source)
        self.assertIn("source_frame,\n                    Time(),", source)

    def test_search_is_one_bounded_rotation(self) -> None:
        source = (SCRIPTS / "pick_place_controller.py").read_text(
            encoding="utf-8"
        )
        for expression in (
            "0.5 * math.pi",
            "math.pi",
            "1.5 * math.pi",
            "2.0 * math.pi",
        ):
            self.assertIn(expression, source)
        self.assertIn('self._publish_status("OBJECT_NOT_FOUND")', source)
        self.assertIn("self.search_exhausted = True", source)

    def test_sim_payload_follows_measured_tool_pose(self) -> None:
        source = (SCRIPTS / "pick_place_controller.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("forward_tool_position", source)
        self.assertIn("SetEntityPose.Request()", source)
        self.assertIn('request.entity.type = 2', source)
        launch = (PACKAGE / "launch" / "gazebo.launch.py").read_text()
        self.assertIn("ros_gz_interfaces/srv/SetEntityPose", launch)

    def test_dashboard_exposes_operator_controls(self) -> None:
        source = (SCRIPTS / "arm_dashboard.py").read_text(encoding="utf-8")
        for value in (
            "/api/state",
            "/api/command",
            "/api/target",
            "/api/manual",
            "/camera.jpg",
            "/pick_place/command",
            "/pick_place/target_color",
        ):
            self.assertIn(value, source)
        controller = (SCRIPTS / "pick_place_controller.py").read_text()
        self.assertIn('command == "START"', controller)
        self.assertIn('command == "STOP"', controller)
        self.assertIn('command == "RESET"', controller)
        self.assertIn("_manual_arm_callback", controller)
        self.assertIn("_manual_gripper_callback", controller)
        self.assertIn("_reset_sim_target", controller)
        self.assertIn("RESET ARM + CUBE", source)

    def test_full_pick_place_launch_starts_automatically(self) -> None:
        launch = (PACKAGE / "launch" / "color_pick_place.launch.py").read_text()
        self.assertIn('default_value="true"', launch)

    def test_grasp_assist_cube_cannot_bounce_between_pose_updates(self) -> None:
        world = (PACKAGE / "worlds" / "color_pick_place.sdf").read_text()
        self.assertEqual(world.count("<gravity>false</gravity>"), 3)
        config = (PACKAGE / "config" / "color_tracker.yaml").read_text()
        self.assertIn("place_z: 0.043", config)

    def test_yaml_files_parse(self) -> None:
        for file in (PACKAGE / "config").glob("*.yaml"):
            with self.subTest(file=file.name):
                with file.open(encoding="utf-8") as handle:
                    self.assertIsNotNone(yaml.safe_load(handle))


class UrdfTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.root = parse(PACKAGE / "urdf" / "printable_6dof_arm.urdf")
        cls.links = {
            link.attrib["name"]: link for link in cls.root.findall("link")
        }
        cls.joints = {
            joint.attrib["name"]: joint for joint in cls.root.findall("joint")
        }

    def test_cad_arm_axes_and_gripper(self) -> None:
        self.assertEqual(
            [self.joints[name].attrib["type"] for name in ARM_JOINTS],
            ["continuous", "revolute", "revolute", "revolute"],
        )
        self.assertEqual(self.joints["left_finger_joint"].attrib["type"], "revolute")
        self.assertEqual(self.joints["right_finger_joint"].attrib["type"], "revolute")
        self.assertIsNone(self.joints["right_finger_joint"].find("mimic"))
        self.assertEqual(
            self.joints["left_finger_joint"].find("axis").attrib["xyz"],
            self.joints["right_finger_joint"].find("axis").attrib["xyz"],
        )

    def test_link_tree_is_connected_and_acyclic(self) -> None:
        children: dict[str, list[str]] = collections.defaultdict(list)
        child_parent: dict[str, str] = {}
        for joint in self.joints.values():
            parent = joint.find("parent").attrib["link"]
            child = joint.find("child").attrib["link"]
            self.assertIn(parent, self.links)
            self.assertIn(child, self.links)
            self.assertNotIn(child, child_parent, f"{child} has two parents")
            child_parent[child] = parent
            children[parent].append(child)

        roots = set(self.links) - set(child_parent)
        self.assertEqual(roots, {"world"})
        visited: set[str] = set()
        stack = ["world"]
        while stack:
            link = stack.pop()
            self.assertNotIn(link, visited, f"Cycle detected at {link}")
            visited.add(link)
            stack.extend(children[link])
        self.assertEqual(visited, set(self.links))

    def test_dynamic_links_have_positive_inertia(self) -> None:
        for name, link in self.links.items():
            if name in {
                "world",
                "base_footprint",
                "tool_tip",
                "camera_optical_frame",
            }:
                continue
            with self.subTest(link=name):
                inertial = link.find("inertial")
                self.assertIsNotNone(inertial)
                self.assertGreater(float(inertial.find("mass").attrib["value"]), 0)
                inertia = inertial.find("inertia").attrib
                for diagonal in ("ixx", "iyy", "izz"):
                    self.assertGreater(float(inertia[diagonal]), 0)

    def test_joint_limits_are_ordered_and_finite(self) -> None:
        for name in ARM_JOINTS + ["left_finger_joint", "right_finger_joint"]:
            with self.subTest(joint=name):
                limit = self.joints[name].find("limit").attrib
                if name != "joint1_base_yaw":
                    lower = float(limit["lower"])
                    upper = float(limit["upper"])
                    self.assertTrue(math.isfinite(lower) and math.isfinite(upper))
                    self.assertLess(lower, upper)
                self.assertGreater(float(limit["effort"]), 0)
                self.assertGreater(float(limit["velocity"]), 0)

    def test_ros2_control_joint_set(self) -> None:
        control = self.root.find("ros2_control")
        controlled = {joint.attrib["name"] for joint in control.findall("joint")}
        self.assertEqual(
            controlled,
            set(ARM_JOINTS + ["left_finger_joint", "right_finger_joint"]),
        )
        plugin = control.find("./hardware/plugin")
        self.assertEqual(plugin.text, "gz_ros2_control/GazeboSimSystem")

    def test_camera_and_gazebo_control_plugins(self) -> None:
        text = (PACKAGE / "urdf" / "printable_6dof_arm.urdf").read_text()
        self.assertIn('sensor name="base_camera" type="camera"', text)
        self.assertIn("/arm_camera/image_raw", text)
        self.assertIn("/arm_camera/camera_info", text)
        self.assertIn("libgz_ros2_control-system.so", text)
        camera_joint = self.joints["camera_mount_joint"]
        self.assertEqual(
            camera_joint.find("parent").attrib["link"], "camera_mount_link"
        )
        bracket_joint = self.joints["camera_bracket_joint"]
        self.assertEqual(
            bracket_joint.find("parent").attrib["link"], "base_yaw_link"
        )
        self.assertEqual(
            bracket_joint.find("child").attrib["link"], "camera_mount_link"
        )
        self.assertEqual(
            len(self.links["camera_mount_link"].findall("visual")), 2
        )

    def test_gripper_gap_matches_the_40_mm_cube(self) -> None:
        left_y = float(
            self.joints["left_finger_joint"]
            .find("origin")
            .attrib["xyz"]
            .split()[1]
        )
        right_y = float(
            self.joints["right_finger_joint"]
            .find("origin")
            .attrib["xyz"]
            .split()[1]
        )
        self.assertAlmostEqual(left_y, 0.040)
        self.assertAlmostEqual(right_y, -0.040)
        config = yaml.safe_load(
            (PACKAGE / "config" / "color_tracker.yaml").read_text()
        )["pick_place_controller"]["ros__parameters"]
        self.assertAlmostEqual(config["closed_gripper_position"], 0.045)
        self.assertLess(config["closed_gripper_position"], 0.10)

    def test_robot_uses_only_user_cad_meshes(self) -> None:
        mesh_names = [mesh.attrib["filename"] for mesh in self.root.findall(".//mesh")]
        self.assertTrue(mesh_names)
        self.assertTrue(all("/meshes/cad/" in name for name in mesh_names))
        self.assertFalse(any("/meshes/visual/" in name for name in mesh_names))

    def test_pedestal_and_upright_base_are_in_the_correct_order(self) -> None:
        pedestal_mesh = self.links["base_link"].find(
            "./visual/geometry/mesh"
        )
        upright_mesh = self.links["base_yaw_link"].find(
            "./visual/geometry/mesh"
        )
        self.assertTrue(pedestal_mesh.attrib["filename"].endswith("base.stl"))
        self.assertTrue(upright_mesh.attrib["filename"].endswith("waist.stl"))
        # Both imported parts are normalized into their assembled ROS frames;
        # added visual rotations would break the reference stack.
        self.assertIsNone(self.links["base_link"].find("./visual/origin"))
        self.assertIsNone(self.links["base_yaw_link"].find("./visual/origin"))
        world_mount = self.joints["world_mount_joint"]
        self.assertEqual(world_mount.attrib["type"], "fixed")
        self.assertEqual(world_mount.find("parent").attrib["link"], "world")
        self.assertEqual(
            world_mount.find("child").attrib["link"], "base_footprint"
        )
        base_yaw_z = float(
            self.joints["joint1_base_yaw"]
            .find("origin")
            .attrib["xyz"]
            .split()[2]
        )
        shoulder_xyz = [
            float(value)
            for value in self.joints["joint2_shoulder_pitch"]
            .find("origin")
            .attrib["xyz"]
            .split()
        ]
        self.assertAlmostEqual(base_yaw_z, 0.05675)
        self.assertAlmostEqual(shoulder_xyz[0], -0.0129)
        self.assertAlmostEqual(shoulder_xyz[1], -0.0072)
        self.assertAlmostEqual(shoulder_xyz[2], 0.0403)


class MeshTests(unittest.TestCase):
    def test_expected_cad_meshes_have_no_open_edges(self) -> None:
        expected = {
            "base.stl",
            "waist.stl",
            "shoulder_arm.stl",
            "elbow_left.stl",
            "elbow_right.stl",
            "wrist_arm.stl",
            "gripper_palm.stl",
            "left_finger.stl",
            "right_finger.stl",
        }
        mesh_dir = PACKAGE / "meshes" / "cad"
        self.assertEqual({path.name for path in mesh_dir.glob("*.stl")}, expected)

        for path in mesh_dir.glob("*.stl"):
            with self.subTest(mesh=path.name):
                vertices = stl_vertices(path)
                self.assertEqual(len(vertices) % 3, 0)
                self.assertGreater(len(vertices) // 3, 100)
                edges: collections.Counter[
                    tuple[
                        tuple[float, float, float],
                        tuple[float, float, float],
                    ]
                ] = collections.Counter()
                for index in range(0, len(vertices), 3):
                    a, b, c = vertices[index : index + 3]
                    self.assertNotEqual(a, b)
                    self.assertNotEqual(b, c)
                    self.assertNotEqual(c, a)
                    for edge in ((a, b), (b, c), (c, a)):
                        edges[tuple(sorted(edge))] += 1
                self.assertTrue(edges)
                self.assertTrue(
                    all(count >= 2 and count % 2 == 0 for count in edges.values()),
                    "The imported CAD must not contain boundary edges",
                )

    def test_mesh_scale_is_millimetres(self) -> None:
        vertices = stl_vertices(PACKAGE / "meshes" / "cad" / "base.stl")
        dimensions = [
            max(v[axis] for v in vertices) - min(v[axis] for v in vertices)
            for axis in range(3)
        ]
        width, depth, height = dimensions
        self.assertAlmostEqual(width, 164.63, delta=0.02)
        self.assertAlmostEqual(depth, 172.92, delta=0.02)
        self.assertAlmostEqual(height, 56.75, delta=0.02)

    def test_cad_importer_and_sources_exist(self) -> None:
        project = PACKAGE.parents[2]
        self.assertTrue((project / "tools" / "import_user_cad.py").is_file())
        self.assertGreaterEqual(len(list((project / "CAD_FILES").glob("*.stl"))), 9)


class KinematicsTests(unittest.TestCase):
    def test_pick_and_place_waypoints(self) -> None:
        waypoints = [
            (0.260, 0.020, 0.025),
            (0.260, 0.020, 0.090),
            (0.260, 0.020, 0.090),
            (0.200, -0.180, 0.090),
            (0.200, -0.180, 0.040),
        ]
        for waypoint in waypoints:
            with self.subTest(waypoint=waypoint):
                joints = solve_topdown_ik(*waypoint)
                self.assertTrue(within_limits(joints, ARM_JOINT_LIMITS))
                actual = forward_tool_position(joints)
                self.assertLess(position_error(waypoint, actual), 1e-9)
                self.assertGreater(
                    joints[2],
                    0.0,
                    "tabletop waypoints must stay on one elbow branch",
                )

    def test_unreachable_point_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            solve_topdown_ik(0.70, 0.0, 0.20)

    def test_target_behind_base_is_supported_by_continuous_yaw(self) -> None:
        joints = solve_topdown_ik(-0.20, 0.0, 0.04)
        expected_yaw = math.pi - math.asin(ArmGeometry().shoulder_y / 0.20)
        self.assertAlmostEqual(joints[0], expected_yaw)
        self.assertLess(
            position_error(
                (-0.20, 0.0, 0.04), forward_tool_position(joints)
            ),
            1e-9,
        )

    def test_geometry_matches_measured_cad_pivots(self) -> None:
        geometry = ArmGeometry()
        self.assertAlmostEqual(geometry.shoulder_z, 0.097)
        self.assertAlmostEqual(geometry.shoulder_x, -0.0129)
        self.assertAlmostEqual(geometry.shoulder_y, -0.0072)
        self.assertAlmostEqual(geometry.upper_arm, 0.120)
        self.assertAlmostEqual(geometry.forearm_to_wrist, 0.204)
        self.assertAlmostEqual(geometry.wrist_to_tip, 0.269)


class ConfigurationTests(unittest.TestCase):
    def test_controller_joint_names_match_urdf(self) -> None:
        config = yaml.safe_load(
            (PACKAGE / "config" / "arm_controllers.yaml").read_text()
        )
        self.assertEqual(config["arm_controller"]["ros__parameters"]["joints"], ARM_JOINTS)
        self.assertEqual(
            config["gripper_controller"]["ros__parameters"]["joints"],
            ["left_finger_joint", "right_finger_joint"],
        )

    def test_servo_calibration_shape_and_channels(self) -> None:
        config = yaml.safe_load(
            (PACKAGE / "config" / "servo_calibration.yaml").read_text()
        )["servo_bridge"]["ros__parameters"]
        names = config["joint_names"]
        self.assertEqual(len(names), 5)
        for key in (
            "channels",
            "offsets_deg",
            "directions",
            "scales_deg_per_unit",
            "servo_min_deg",
            "servo_max_deg",
            "command_min",
            "command_max",
            "max_command_rate_per_s",
        ):
            self.assertEqual(len(config[key]), len(names), key)
        self.assertEqual(len(set(config["channels"])), 5)
        self.assertTrue(config["dry_run"])

    def test_world_has_three_colours_and_tray(self) -> None:
        world = parse(PACKAGE / "worlds" / "color_pick_place.sdf")
        models = {model.attrib["name"] for model in world.findall("./world/model")}
        self.assertTrue(
            {
                "red_pick_cube",
                "green_pick_cube",
                "blue_pick_cube",
                "placement_tray",
            }.issubset(models)
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
