cmake_minimum_required(VERSION 2.8.3)
project(lidar_cone_detection)

## Compile as C++11, supported in ROS Kinetic and newer
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
)

add_executable(${PROJECT_NAME} src/LidarConeDetectionNODE.cpp src/LidarConeDetection.cpp src/LidarConeDetectionSynch.cpp)

target_link_libraries(lidar_cone_detection
        ${catkin_LIBRARIES}
        )
