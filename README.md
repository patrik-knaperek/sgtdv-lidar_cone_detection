# lidar_cone_detection

### Setup

1. connect to LIDAR according to [Getting Started with the Velodyne VLP16](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
2. move to project home directory e.g. `cd ~/ros_implementation/`
3. run following commands
    * `source devel/setup.sh`
    * `git clone https://github.com/ros-drivers/velodyne.git src/velodyne`
    * `rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y`
    * `catkin_make --only-pkg-with-deps sgtdv_msgs`
    * `catkin_make --only-pkg-with-deps velodyne lidar_cone_detection`
    * `roslaunch lidar_cone_detection lidar_cone_detection.launch`

To see data from LIDAR as well as detected cones, make sure that **DEBUG_STATE** in _SGT_Macros.h_ is set to **1** and run :

`rosrun rviz rviz -d src/lidar_cone_detection/lidar_cone_detection.rviz`