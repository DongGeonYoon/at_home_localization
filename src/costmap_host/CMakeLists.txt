cmake_minimum_required(VERSION 3.8)
project(costmap_host)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(costmap_host src/costmap_host.cpp)
ament_target_dependencies(costmap_host rclcpp nav2_costmap_2d)

add_library(fresh_voxel_layer SHARED src/fresh_voxel_layer.cpp)
ament_target_dependencies(fresh_voxel_layer nav2_costmap_2d pluginlib sensor_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_host_plugins.xml)

install(TARGETS costmap_host DESTINATION lib/${PROJECT_NAME})
install(TARGETS fresh_voxel_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
