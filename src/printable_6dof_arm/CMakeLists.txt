cmake_minimum_required(VERSION 3.8)
project(printable_6dof_arm)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/arm_kinematics.py
    scripts/arm_dashboard.py
    scripts/color_tracker.py
    scripts/color_tracker_hw_test.py
    scripts/camera_calibration_hw.py
    scripts/pixel_to_tabletop_hw.py
    scripts/pick_place_controller.py
    scripts/servo_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
