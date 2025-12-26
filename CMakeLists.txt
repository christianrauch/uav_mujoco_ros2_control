cmake_minimum_required(VERSION 3.16)
project(uav_mujoco_ros2_control)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

# find_package(lodepng REQUIRED)

find_package(mujoco_vendor REQUIRED)
find_package(glfw3 REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_link_options("-Wl,-z,relro,-z,now,-z,defs")
endif()

add_library(simulate OBJECT
  ${mujoco_vendor_DIR}/../../../opt/mujoco_vendor/simulate/simulate.cc
  ${mujoco_vendor_DIR}/../../../opt/mujoco_vendor/simulate/glfw_adapter.cc # ??
  ${mujoco_vendor_DIR}/../../../opt/mujoco_vendor/simulate/platform_ui_adapter.cc
  ${mujoco_vendor_DIR}/../../../opt/mujoco_vendor/simulate/glfw_dispatch.cc
)
target_include_directories(simulate PUBLIC
  ${mujoco_vendor_DIR}/../../../opt/mujoco_vendor/simulate
)
target_link_libraries(simulate PUBLIC
  mujoco::mujoco
  lodepng
)
set_property(TARGET simulate PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(uav_mujoco_system SHARED
  src/mujoco_system.cpp
)
# target_include_directories(uav_mujoco_system PUBLIC
#   # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   # $<INSTALL_INTERFACE:include>
#   ${mujoco_vendor_DIR}/../../../opt/mujoco_vendor/simulate
# )

# mujoco_vendor_DIR:PATH=/opt/ros/jazzy/share/mujoco_vendor/cmake

# /opt/ros/jazzy/opt/mujoco_vendor/simulate/simulate.cc
# /opt/ros/jazzy/opt/mujoco_vendor/simulate/simulate.h


target_link_libraries(uav_mujoco_system PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  realtime_tools::realtime_tools
  tf2_ros::tf2_ros
  ${tf2_msgs_TARGETS}
  mujoco::mujoco
  glfw # ??
  simulate
)

install(TARGETS uav_mujoco_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(hardware_interface uav_mujoco_ros2_control_system.xml)

ament_package()
