cmake_minimum_required(VERSION 3.16)
project(uav_mujoco_ros2_control)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

add_library(uav_mujoco_system SHARED
  src/mujoco_system.cpp
)

target_link_libraries(uav_mujoco_system PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  mujoco::mujoco
  glfw
)

install(TARGETS uav_mujoco_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(hardware_interface uav_mujoco_ros2_control_system.xml)

ament_package()
