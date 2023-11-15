cmake_minimum_required(VERSION 3.8)
project(sauvc_sim)

find_package(
  ament_cmake REQUIRED
  rclcpp REQUIRED
	gazebo_ros REQUIRED
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
