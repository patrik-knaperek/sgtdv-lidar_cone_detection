cmake_minimum_required(VERSION 3.0.7)
project(lidar_cone_detection)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  sgtdv_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp
  CATKIN_DEPENDS std_msgs
  CATKIN_DEPENDS sgtdv_msgs
  CATKIN_DEPENDS visualization_msgs
  CATKIN_DEPENDS sensor_msgs
  CATKIN_DEPENDS pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/lidar_cone_detection_node.cpp src/lidar_cone_detection.cpp)

target_link_libraries(lidar_cone_detection
  ${catkin_LIBRARIES}
)
