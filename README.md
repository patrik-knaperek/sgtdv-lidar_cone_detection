# **LidarConeDetection package**

___

&copy; **SGT Driverless**

**Authors:** Juraj Krasňanský, Matej Dudák, Lukáš Lánik

**Objective:** Cone detection and position estimation from Velodyne lidar.

___

`lidar_cone_detection` node processes pointcloud data from `velodyne_driver` into estimated cone positions.  
It works in 3 steps:
- pointcloud filtering by intensity and Cartesian coordinates
- pointcloud euclidean clustering (based of euclidean distance of points)
- cone center estimation (centroid approximation and Newton-Gauss estimation)

**Note**: This version of algorithm cannot distinguish a cone from any other type of object

### Related packages
* `velodyne`
* `ros_pcl`

### Setup

1. connect to LIDAR according to [Getting Started with the Velodyne VLP16](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
2. move to project home directory e.g. `cd ~/ros_implementation/`
3. run:
```sh
    $ cd ${SGT_ROOT}/ros_implementation
    $ source devel/setup.sh
    $ git clone https://github.com/ros-drivers/velodyne.git src/velodyne
    $ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

## Compilation
The following packages have to be built at first:
  * `sgtdv_msgs`
  * `velodyne`

In folder `ros_implementation/src/` run:
```sh
    $ cd ${SGT_ROOT}/ros_implementation
    $ catkin build lidar_cone_detection
```

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

### Configuration
 * [`lidar_cone_detection.yaml`](./params/lidar_cone_detection.yaml):
    - `pcl_cluster/n_points/min` : minimal number of points in cluster extracted from pointcloud
    - `pcl_cluster/n_points/min` : maximal number of points in cluster extracted from pointcloud
    - `pcl_cluster/radius` : radius of point cluster extraction [m]
    - `mean_cone_radius` : mean value of cone radius [m]
    - `pcl_filter/intensity/min` : minimum point intensity value to pass the filter 
    - `pcl_filter/intensity/max` : maximum point intensity value to pass the filter
    - `pcl_filter/x/min` : minimum point x-coordinate value to pass the filter [m]
    - `pcl_filter/x/max` : maximum point x-coordinate value to pass the filter [m]
    - `pcl_filter/y/min` : minimum point y-coordinate value to pass the filter [m]
    - `pcl_filter/y/max` : maximum point y-coordinate value to pass the filter [m]
    - `pcl_filter/z/min` : minimum point z-coordinate value to pass the filter [m]
    - `pcl_filter/z/max` : maximum point z-coordinate value to pass the filter [m]

### RViz visualization
In new terminal run:
```sh
    $ source ${SGT_ROOT}/ros_implementation/devel/setup.bash
    $ roslaunch data_visualization data_visualization_lidar.launch
```
