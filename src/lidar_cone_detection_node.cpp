/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Matej Dudák
/*****************************************************/


#include "lidar_cone_detection_ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_cone_detection");
  ros::NodeHandle handle;
  LidarConeDetectionROS ros_obj(handle);

  ros::spin();

  return 0;
}