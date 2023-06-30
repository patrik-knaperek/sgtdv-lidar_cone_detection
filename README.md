# **LidarConeDetection package**

___

&copy; **SGT Driverless**

**Authors:** Juraj Krasňanský, Matej Dudák

**Objective:** Cone detection and position estimation from Velodyne lidar.

___

### Related packages
* `velodyne`

### Setup

1. connect to LIDAR according to [Getting Started with the Velodyne VLP16](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
2. move to project home directory e.g. `cd ~/ros_implementation/`
3. run:
```
    $ source devel/setup.sh
    $ git clone https://github.com/ros-drivers/velodyne.git src/velodyne
    $ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

## Compilation
The following packages have to be built at first:
  * `sgtdv_msgs`
  * `velodyne`

In folder `ros_implementation/src/` run:
```
$ catkin build lidar_cone_detection
```

### Compilation configuration
 * `LidarConeDetection.h`:
    - `CONE_CLUSTER_MIN_POINTS` : minimal number of points in cluster extracted from pointcloud
    - `CONE_CLUSTER_MAX_POINTS` : maximal number of points in cluster extracted from pointcloud
    - `CONE_CLUSTER_RADIUS` : radius of point cluster extraction [m]
    - `CONE_RADIUS` : mean value of cone radius [m]
    - `CONE_INTENSITY_MIN` : minimum point intensity value to pass the filter 
    - `CONE_INTENSITY_MAX` : maximum point intensity value to pass the filter
    - `CONE_X_MIN` : minimum point x-coordinate value to pass the filter [cm]
    - `CONE_X_MAX` : maximum point x-coordinate value to pass the filter [cm]
    - `CONE_Y_MIN` : minimum point y-coordinate value to pass the filter [cm]
    - `CONE_Y_MAX` : maximum point y-coordinate value to pass the filter [cm]
    - `CONE_Z_MIN` : minimum point z-coordinate value to pass the filter [cm]
    - `CONE_Z_MAX` : maximum point z-coordinate value to pass the filter [cm]

## Launch
```
    $ source ros_implementation/devel/setup.bash
    $ roslaunch lidar_cone_detection lidar_cone_detection.launch
```
### RViz visualization
In new terminal run:
```
    $ source ros_implementation/devel/setup.bash
    $ roslaunch sensors_visualizator sensors_visualizator_lidar.launch
```
