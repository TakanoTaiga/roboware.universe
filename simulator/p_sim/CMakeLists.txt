cmake_minimum_required(VERSION 3.5)
project(p_sim)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_p_sim SHARED
  DIRECTORY src
)

target_include_directories(target_p_sim
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_p_sim
  PLUGIN "p_sim::PSIMNode"
  EXECUTABLE node
)

ament_auto_package(
  INSTALL_TO_SHARE
)