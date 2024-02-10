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
#include <sgtdv_msgs/DebugState.h>

//values are in meters
#define CONE_CLUSTER_MIN_POINTS 3
#define CONE_CLUSTER_MAX_POINTS 500
#define CONE_CLUSTER_RADIUS 0.3 //30 cm
#define CONE_RADIUS 0.11 // 11cm
#define EPSILON_ERROR 0.01 // 1cm

#define CONE_INTENSITY_MIN 40
#define CONE_INTENSITY_MAX 250
#define CONE_X_MIN 0.75
#define CONE_X_MAX 30
#define CONE_Y_MIN (-10)
#define CONE_Y_MAX 10
#define CONE_Z_MIN 0.0
#define CONE_Z_MAX 0.3


class LidarConeDetection {
public:
    LidarConeDetection();

    ~LidarConeDetection();

    void SetPublisher(ros::Publisher publisher);

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) const;

	// Eigen::Vector3f deltaVec(pcl::PointIndices indices, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered, Eigen::Vector3f coneCenter);

#ifdef SGT_DEBUG_STATE
    void SetVisDebugPublisher(ros::Publisher publisher) { vis_debug_publisher_ = publisher; }
#endif

private:
    ros::Publisher publisher_;

	

#ifdef SGT_DEBUG_STATE
    ros::Publisher vis_debug_publisher_;
#endif

};
