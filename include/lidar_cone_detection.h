/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Matej Dudák, Lukáš Lánik
/*****************************************************/

# pragma once

/* C++ */
#include <Eigen/Eigen>

/* ROS */
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

/* SGT */
#include <sgtdv_msgs/Point2DStampedArr.h>
#include "SGT_Utils.h"

class LidarConeDetection
{
public:
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

public:
  explicit LidarConeDetection() = default;
  ~LidarConeDetection() = default;

  sgtdv_msgs::Point2DStampedArr update(const sensor_msgs::PointCloud2::ConstPtr &msg);

  void setParams(const Params& params)
  {
    params_ = params;
  };

private:
  Eigen::Vector2f calculateGradient(const pcl::PointXYZ &point, const pcl::PointIndices &cluster_indices,
                                    const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud_ptr) const;

  void newtonGaussEstimation(const sgtdv_msgs::Point2DStampedArrPtr &cones, 
                            std::vector<pcl::PointIndices> &cluster_indices, 
                            const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_ptr) const;

  Eigen::Matrix2f calculateHessian(const pcl::PointXYZ &point, const pcl::PointIndices &cluster_indices, 
                                  const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_ptr) const;

  Params params_;

  static constexpr float EPSILON_ERROR = 0.05; // 5cm
  static constexpr int ITER_LIMIT = 10;
};
