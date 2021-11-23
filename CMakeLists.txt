cmake_minimum_required(VERSION 3.5)
project(points_map_loader)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(points_map_loader_node SHARED src/points_map_loader_node.cpp)
rclcpp_components_register_node(points_map_loader_node
  PLUGIN "PointsMapLoader"
  EXECUTABLE points_map_loader
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_auto_package(INSTALL_TO_SHARE launch rviz)
