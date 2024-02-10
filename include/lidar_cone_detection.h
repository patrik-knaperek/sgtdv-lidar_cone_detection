/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Matej Dudák, Lukáš Lánik
/*****************************************************/

# pragma once

/* C++ */
#include <cmath>
#include <vector>

/* ROS */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>

/* SGT */
#include <sgtdv_msgs/Point2DStampedArr.h>
#include "../../SGT_Macros.h"
#include "../../SGT_Utils.h"
#include <sgtdv_msgs/DebugState.h>

class LidarConeDetection
{
public:
  LidarConeDetection(ros::NodeHandle& nh);

  ~LidarConeDetection() = default;

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
  };

private:
  ros::Publisher publisher_;
  ros::Subscriber pcl_sub_;
	
  Params params_;

#ifdef SGT_DEBUG_STATE
  ros::Publisher vis_debug_publisher_;
#endif
};
