cmake_minimum_required(VERSION 3.16)
project(fr3wml_industrial_bt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fairino_bridge REQUIRED)
find_package(industrial_bt_framework REQUIRED)

# ---------- FairinoMoveLBackend plugin ----------
add_library(fr3wml_industrial_bt_backends SHARED
  plugins/fairino_movel_backend.cpp
)
target_include_directories(fr3wml_industrial_bt_backends PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${industrial_bt_framework_INCLUDE_DIRS}
)
ament_target_dependencies(fr3wml_industrial_bt_backends
  rclcpp
  pluginlib
  geometry_msgs
  tf2
  tf2_geometry_msgs
  yaml_cpp_vendor
  fairino_bridge
  industrial_bt_framework
)
target_link_libraries(fr3wml_industrial_bt_backends yaml-cpp)

pluginlib_export_plugin_description_file(
  industrial_bt_framework plugins/fr3wml_industrial_bt_plugins.xml)

# ---------- install ----------
install(TARGETS fr3wml_industrial_bt_backends
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch bt_trees config meshes urdf rviz doc plugins tools
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
