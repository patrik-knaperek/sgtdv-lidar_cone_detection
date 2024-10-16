/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Matej Dudák, Lukáš Lánik
/*****************************************************/

# pragma once

/* ROS */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

/* SGT */
#include <sgtdv_msgs/Point2DStampedArr.h>
#include "SGT_Macros.h"
#include "SGT_Utils.h"

class LidarConeDetection
{
public:
  explicit LidarConeDetection(ros::NodeHandle& nh);
  ~LidarConeDetection() = default;

private:
  void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) const;

  struct Params
  {
    Utils::Range<double> cluster_points;
    Utils::Range<double> intensity;
    Utils::Range<double> x_range;
    Utils::Range<double> y_range;
    Utils::Range<double> z_range;
    double cluster_radius;
    double mean_cone_radius;
  } params_;

  ros::Publisher publisher_;
  ros::Subscriber pcl_sub_;
	
#ifdef SGT_DEBUG_STATE
  ros::Publisher vis_debug_publisher_;
#endif
};
