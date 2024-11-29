/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

/* SGT */
#include <sgtdv_msgs/DebugState.h>

/* Header */
#include <lidar_cone_detection_ros.h>

LidarConeDetectionROS::LidarConeDetectionROS(ros::NodeHandle& nh)
	/* ROS Interface initialization */
	: cones_pub_(nh.advertise<sgtdv_msgs::Point2DStampedArr>("lidar/cones", 1))
	, pcl_sub_(nh.subscribe("velodyne_points", 1, &LidarConeDetectionROS::lidarCallback, this))
#ifdef SGT_DEBUG_STATE
	, vis_debug_pub_(nh.advertise<sgtdv_msgs::DebugState>("lidar/debug_state", 2))
#endif
{
	loadParams(nh);
}

void LidarConeDetectionROS::loadParams(const ros::NodeHandle& nh)
{
	LidarConeDetection::Params params;

	/* Load ROS parameters from server */
	Utils::loadParam(nh, "pcl_filter/x/min", &params.x_range.min);
	Utils::loadParam(nh, "pcl_filter/x/max", &params.x_range.max);
	Utils::loadParam(nh, "pcl_filter/y/min", &params.y_range.min);
	Utils::loadParam(nh, "pcl_filter/y/max", &params.y_range.max);
	Utils::loadParam(nh, "pcl_filter/z/min", &params.z_range.min);
	Utils::loadParam(nh, "pcl_filter/z/max", &params.z_range.max);
	Utils::loadParam(nh, "pcl_cluster/n_points/min", &params.cluster_points.min);
	Utils::loadParam(nh, "pcl_cluster/n_points/max", &params.cluster_points.max);
	Utils::loadParam(nh, "pcl_filter/intensity/min", &params.intensity.min);
	Utils::loadParam(nh, "pcl_filter/intensity/max", &params.intensity.max);
	Utils::loadParam(nh, "pcl_cluster/radius", &params.cluster_radius);
	Utils::loadParam(nh, "mean_cone_radius", &params.mean_cone_radius);

	lidar_cone_detection_obj_.setParams(params);
}

void LidarConeDetectionROS::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
#ifdef SGT_DEBUG_STATE
  sgtdv_msgs::DebugState state;
  state.stamp = ros::Time::now();
  state.working_state = 1;
  vis_debug_pub_.publish(state);
#endif

	const auto cone_array = lidar_cone_detection_obj_.update(msg);
	cones_pub_.publish(cone_array);

#ifdef SGT_DEBUG_STATE
  state.num_of_cones = cone_array.points.size();
  state.stamp = ros::Time::now();
  state.working_state = 0;
  vis_debug_pub_.publish(state);
#endif
}
