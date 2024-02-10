/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Matej Dudák
/*****************************************************/


#include "../include/lidar_cone_detection.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidarConeDetection");
  ros::NodeHandle handle;
  LidarConeDetection lidar_cone_detection(handle);

  ros::spin();

  return 0;
}