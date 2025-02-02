# **lidar_cone_detection package**

___

&copy; **SGT Driverless**

**Authors:** Juraj Krasňanský, Matej Dudák, Lukáš Lánik

**Objective:** Cone detection and position estimation from Velodyne lidar.

___

## Overview

`lidar_cone_detection` node processes pointcloud data from `velodyne` ROS wrapper into estimated cone positions.  
It works in 3 steps:
1. pointcloud filtering by intensity and Cartesian coordinates
2. pointcloud euclidean clustering (based of euclidean distance of points)
3. cone center estimation (centroid approximation based on average position vector of points in the cluster and mean cone radius)

**Note**: This version of algorithm cannot distinguish a cone from any other type of object

### ROS Interface

**Subscribed topics**
* `/velodyne_points` [[`sensor_msgs/PointCLoud2`](/opt/ros/noetic/share/sensor_msgs/msg/PointCloud2.msg)] : raw LiDAR data

**Published topics**
* `/lidar/points` [[`sgtdv_msgs/Point2DStamedArr`](../sgtdv_msgs/msg/Point2DStampedArr.msg)] : LiDAR cone detections in `lidar` frame

*If `SGT_DEBUG_STATE` macro enabled*
* `/lidar_cone_detection/debug_state` [[`sgtdv_msgs/DebugState`](../sgtdv_msgs/msg/DebugState.msg)] : node lifecycle information (active/inactive, number of detected cones)

**Parameters**
* `/pcl_cluster/n_points/min` : minimal number of points in cluster extracted from pointcloud
* `/pcl_cluster/n_points/max` : maximal number of points in cluster extracted from pointcloud
* `/pcl_cluster/radius` : radius of point cluster extraction [m]
* `/mean_cone_radius` : mean value of cone radius [m]
* `/pcl_filter/intensity/min` : minimum point intensity value to pass the filter 
* `/pcl_filter/intensity/max` : maximum point intensity value to pass the filter
* `/pcl_filter/x/min` : minimum point x-coordinate value to pass the filter [m]
* `/pcl_filter/x/max` : maximum point x-coordinate value to pass the filter [m]
* `/pcl_filter/y/min` : minimum point y-coordinate value to pass the filter [m]
* `/pcl_filter/y/max` : maximum point y-coordinate value to pass the filter [m]
* `/pcl_filter/z/min` : minimum point z-coordinate value to pass the filter [m]
* `/pcl_filter/z/max` : maximum point z-coordinate value to pass the filter [m]

### Related packages
* [`velodyne`](../velodyne/README.md)
* `ros_pcl`

## Installation

### LiDAR setup

1. Connect the computer to LIDAR according to [Getting Started with the Velodyne VLP16](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
2. Open web browser and connect to the IP adress `192.168.1.201` to configure LIDAR behaviour and data being sent using web interface (e. g. RPM, FOV, etc.). (*Here you set physical parameters of the LIDAR scanning - what data is sent from LIDAR to PC LAN interface.*)
3. Set the laser parametres in the [driver's launchfile](../velodyne/velodyne_pointcloud/launch/VLP16_points.launch). (*Here you set data processing parameters of the velodyne ROS driver - what data is sent from PC LAN interface to ROS network.*)

## Compilation
* standalone
```sh
  $ catkin build lidar_cone_detection -DCMAKE_BUILD_TYPE=Release
```
* RC car setup
```sh
  $ source ${SGT_ROOT}/scripts/build_rc.sh
```

### Compilation configuration
* [`SGT_Macros.h`](../SGT_Macros.h) :
  - `SGT_DEBUG_STATE` : publish node lifecycle information

## Launch
```sh
    $ source ${SGT_ROOT}/ros_implementation/devel/setup.bash
    $ roslaunch <name_of_package> <name_of_lanchfile>
```
Available launchfiles:
* `lidar_cone_detection` package
  * [`lidar_cone_detection.launch`](./launch/lidar_cone_detection.launch) : standalone with real sensor
  * [`lidar_cone_detection_offline.launch`](./launch/lidar_cone_detection_offline.launch) : standalone with rosbag data
      - In another terminal, `cd` to directory containing desired rosbags, then run `$ rosbag play -l <rosbag_name>`
* `fusion` package
  * [`fusion_rc.launch`](../fusion/launch/fusion_rc.launch) : along with `camera_driver` and `fusion` nodes (requires them to be built first)
* `master` package
  * [`rc.launch`](../master/launch/rc.launch) : along with complete pipeline

### Launch configuration
 * [`lidar_cone_detection.yaml`](./params/lidar_cone_detection.yaml)
 * [`VLP16_points.launch`](../velodyne/velodyne_pointcloud/launch/VLP16_points.launch)

### RViz visualization
* standalone
```sh
  $ roslaunch data_visualization data_visualization_lidar.launch
```
* RC car setup
```sh
  $ roslaunch data_visualization data_visualization_rc.launch
```

## Diagrams and flowcharts
