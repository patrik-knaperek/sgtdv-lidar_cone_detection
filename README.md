# lidar_cone_detection

### Setup
* move to project home directory e.g. `cd ~/ros_implementation/`
* run following commands
  * `source devel/setup.sh`
  * `git clone https://github.com/ros-drivers/velodyne.git src/velodyne`
  * `rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y`
  * `catkin_make --only-pkg-with-deps velodyne lidar_cone_detection`
  * `roslaunch lidar_cone_detection lidar_cone_detection`

To see it running, make sure that **DEBUG_STATE** in _SGT_Macros.h_ is set to **1** and run `rosrun rviz rviz -d src/lidar_cone_detection/lidar_cone_detection.rviz`