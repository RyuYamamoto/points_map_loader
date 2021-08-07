cmake_minimum_required(VERSION 3.0.2)
project(points_map_loader)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES points_map_loader
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(points_map_loader_node src/points_map_loader_node.cpp)
target_link_libraries(points_map_loader_node ${catkin_LIBRARIES})
