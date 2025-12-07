cmake_minimum_required(VERSION 3.16)
project(mujoco_ros2_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)

find_path(MUJOCO_INCLUDE_DIR mujoco.h)
find_library(MUJOCO_LIBRARY NAMES mujoco)

include_directories(include ${MUJOCO_INCLUDE_DIR})

add_library(mujoco_system SHARED
  src/mujoco_system.cpp
)

target_include_directories(mujoco_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(mujoco_system PUBLIC ${MUJOCO_LIBRARY})

ament_target_dependencies(mujoco_system PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  controller_manager
)

install(TARGETS mujoco_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES resource/mujoco_ros2_control_system.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
