cmake_minimum_required(VERSION 3.8)
project(robot_bring_up)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

add_library(manual_costmap_layer SHARED src/manual_costmap_layer.cpp)
target_compile_features(manual_costmap_layer PUBLIC cxx_std_14)
target_include_directories(manual_costmap_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(manual_costmap_layer
  rclcpp nav2_costmap_2d pluginlib geometry_msgs robot_interfaces)
pluginlib_export_plugin_description_file(nav2_costmap_2d manual_costmap_plugins.xml)

add_library(manual_costmap_panel SHARED src/manual_costmap_panel.cpp)
target_compile_features(manual_costmap_panel PUBLIC cxx_std_14)
ament_target_dependencies(manual_costmap_panel rclcpp pluginlib rviz_common std_srvs)
target_link_libraries(manual_costmap_panel Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common manual_costmap_rviz_plugins.xml)


install(TARGETS manual_costmap_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS manual_costmap_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


install(PROGRAMS scripts/manual_costmap_editor.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(manual_costmap_layer)
ament_export_dependencies(rclcpp nav2_costmap_2d pluginlib geometry_msgs robot_interfaces rviz_common std_srvs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)
