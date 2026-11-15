# Printable Robotic Arm — ROS 2, Gazebo & Vision-Based Pick and Place

This repository contains the `printable_6dof_arm` ROS 2 package for a user-CAD robotic arm. The package metadata describes a four-axis arm with Gazebo/RViz support, camera-based color tracking, and automatic pick-and-place control.

## Project Highlights

- **Robot description:** URDF and Xacro files, CAD meshes, and configuration assets.
- **Simulation and visualization:** launch files for RViz and Gazebo.
- **Vision pipeline:** camera/color-tracking scripts and calibration utilities using OpenCV and ROS image messages.
- **Pick-and-place:** kinematics and controller scripts for color-based object handling.
- **Hardware-oriented utilities:** servo bridge and camera/tabletop calibration launch paths.

## Important Kinematic Note

The checked-in URDF currently models **four arm-motion axes** (base yaw, shoulder pitch, elbow pitch, and wrist pitch), plus **two gripper-finger joints**. The two finger joints open and close the gripper; they are not two additional arm-positioning axes. The repository name contains “6-DOF,” but this distinction matters when describing the manipulator's kinematics.

## Repository Structure

```text
src/printable_6dof_arm/
├── cad/       CAD generation assets and printing notes
├── config/    Controller, camera, vision, and RViz configuration
├── launch/    Display, Gazebo, vision, and pick-and-place launch files
├── meshes/    CAD and visual STL meshes
├── scripts/   Kinematics, dashboard, color tracking, pick/place, servo utilities
├── tests/     Package tests
├── urdf/      URDF and Xacro robot descriptions
└── worlds/    Gazebo world assets
```

## Requirements

- A compatible ROS 2 installation
- `colcon` and `rosdep`
- Gazebo / ROS-Gazebo integration packages compatible with the installed ROS 2 distribution
- OpenCV and `cv_bridge` for the vision pipeline

Install dependencies from the workspace root:

```bash
source /opt/ros/<ros-distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

Run these commands from the repository root (the ROS 2 workspace root):

```bash
source /opt/ros/<ros-distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch Options

After building and sourcing the workspace, the package provides these launch entry points:

```bash
# RViz robot display
ros2 launch printable_6dof_arm display.launch.py

# Gazebo simulation
ros2 launch printable_6dof_arm gazebo.launch.py

# Color-based pick-and-place simulation
ros2 launch printable_6dof_arm color_pick_place.launch.py
```

The package also includes hardware-oriented launch files for camera/vision and servo workflows. Review their configuration and hardware assumptions before using them.

## Safety & Scope

- Start with simulation and verify joint limits, controller behavior, camera calibration, and object detection before connecting physical hardware.
- Hardware launch files may require device paths, camera settings, and servo calibration to be adjusted for the local setup.
- This repository documents a student engineering project; it is not a certified industrial robot controller.

## License

The ROS 2 package metadata declares the Apache-2.0 license. Review the project files and third-party asset terms before redistributing CAD meshes or other included assets.
