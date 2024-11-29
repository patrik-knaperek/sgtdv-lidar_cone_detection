/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/


/* ROS */
#include <ros/ros.h>

/* SGT-DV */
#include <sgtdv_msgs/Point2DStampedArr.h>
#include "SGT_Macros.h"
#include "SGT_Utils.h"
#include <lidar_cone_detection.h>

class LidarConeDetectionROS
{
  public:
    LidarConeDetectionROS(ros::NodeHandle& nh);
    
    ~LidarConeDetectionROS() = default;
      
  private:
    void loadParams(const ros::NodeHandle& nh);

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  LidarConeDetection lidar_cone_detection_obj_;
  
  ros::Publisher cones_pub_;
#ifdef SGT_DEBUG_STATE
  ros::Publisher vis_debug_pub_;
#endif

  ros::Subscriber pcl_sub_;
};