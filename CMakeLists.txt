cmake_minimum_required(VERSION 3.8)
project(sauvc_sim)

find_package(
  ament_cmake REQUIRED
  ament_cmake_python REQUIRED
  rclcpp REQUIRED
  rclpy REQUIRED
	gazebo_ros REQUIRED
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  models
  worlds
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
