/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Matej Dudák, Lukáš Lánik
/*****************************************************/

/* ROS */
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>

/* Header */
#include "lidar_cone_detection.h"

sgtdv_msgs::Point2DStampedArr LidarConeDetection::update(const sensor_msgs::PointCloud2::ConstPtr &msg)
{  
  /* Pointcloud filtering */
  
  pcl::PCLPointCloud2::Ptr cloud = pcl::make_shared<pcl::PCLPointCloud2>();
  pcl::PCLPointCloud2::ConstPtr cloud_ptr(cloud);

  pcl_conversions::toPCL(*msg, *cloud);
  pcl::PassThrough<pcl::PCLPointCloud2> pass_through;
  
  pass_through.setInputCloud(cloud_ptr);
  if(cloud->width > 0)
  {
    //filter data by intensity
    pass_through.setFilterFieldName("intensity");
    pass_through.setFilterLimits(params_.intensity.min, params_.intensity.max);
    pass_through.filter(*cloud);
  }
  if(cloud->width > 0)
  {
    //filter data by X axis (forward distance from lidar sensor)
    pass_through.setFilterFieldName("x");
    pass_through.setFilterLimits(params_.x_range.min, params_.x_range.max);
    pass_through.filter(*cloud);
  }
  if(cloud->width > 0)
  {
    //filter data by y axis (side distance from lidar sensor)
    pass_through.setFilterFieldName("y");
    pass_through.setFilterLimits(params_.y_range.min, params_.y_range.max);
    pass_through.filter(*cloud);
  }

  if(cloud->width > 0)
  {
    //filter data by z axis (vertical distance from lidar sensor)
    pass_through.setFilterFieldName("z");
    pass_through.setFilterLimits(params_.z_range.min, params_.z_range.max);
    pass_through.filter(*cloud);
  }
  
  sgtdv_msgs::Point2DStampedArr cone_array;

  if(cloud->width > 2)
  {
    /**
     * Pointcloud clustering
     * https://pointclouds.org/documentation/classpcl_1_1_euclidean_cluster_extraction.html
     * using Euclidean cluster extraction to get clusters of CONE_CLUSTER_POINTS within CONE_CLUSTER_RADIUS
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromPCLPointCloud2(*cloud, *cloud_filtered);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    std::vector<pcl::PointIndices> cluster_indices;
    tree->setInputCloud(cloud_filtered);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(params_.cluster_radius);
    ec.setMinClusterSize(params_.cluster_points.min);
    ec.setMaxClusterSize(params_.cluster_points.max);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    auto cone_arr_ptr = boost::make_shared<sgtdv_msgs::Point2DStampedArr>(cone_array);
    newtonGaussEstimation(cone_arr_ptr, cluster_indices, cloud_filtered);
    
    for (auto point : cone_arr_ptr->points)
      point.header.stamp = msg->header.stamp;

    cone_array = *cone_arr_ptr;

#pragma region DEPRECATED
    //----------------------- DEPRECATED ----------------------------------

    /**
     * Cone detection
     * Approximate centroid of visible semicricular arc of cone by taking average
     * over all points in point cluser then scale the position vector of the centroid
     * to get the approximate position of cone center
     */
  
    // if(!cluster_indices.empty())
    // {
    //   cone_array.points.reserve(cluster_indices.size());
    //   int i_n = 0;
    //   for(const auto &indices: cluster_indices)
    //   {
    //     pcl::PointXYZ centroid_pos;
        
    //     for (int i: indices.indices)
    //     {
    //       centroid_pos.x += cloud_filtered->points[i].x;
    //       centroid_pos.y += cloud_filtered->points[i].y;
    //     }

    //     centroid_pos.x /= indices.indices.size();
    //     centroid_pos.y /= indices.indices.size();
    //     const double centroid_pos_abs = sqrt(pow(centroid_pos.x, 2) + pow(centroid_pos.y, 2));
    //     const double scale_factor = 1 + 2 * params_.mean_cone_radius / (M_PI * centroid_pos_abs);
        
    //     sgtdv_msgs::Point2DStamped point;
    //     point.x = centroid_pos.x * scale_factor;
    //     point.y = centroid_pos.y * scale_factor;
    //     point.header.frame_id = "lidar";
    //     point.header.seq = i_n++;
    //     point.header.stamp = msg->header.stamp;
    //     cone_array.points.push_back(point);
    //   }
    // }
  #pragma endregion /* DEPRECATED */
  }

  return cone_array;
}

/*
    Newton-Gauss method approximation of a cluster centre
*/
void LidarConeDetection::newtonGaussEstimation(const sgtdv_msgs::Point2DStampedArrPtr &cones, 
                                              std::vector<pcl::PointIndices> &cluster_indices,
                                              const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_ptr) const
{
  //ROS_DEBUG_STREAM("cluster_indices contains " << cluster_indices.size() << " elements");
  if (!cluster_indices.empty())
  {
    cones->points.reserve(cluster_indices.size());
    int i_n = 0; // id for each cone cluster

    for (const auto &indices: cluster_indices)
    {

      ROS_DEBUG("Iterating through cluster");
      pcl::PointXYZ centroid_pos, center_estimate, iter_ioint;
      int n = 0; // number of points in cluster
      for (const int i : indices.indices)
      {
        centroid_pos.x += cloud_ptr->points[i].x;
        centroid_pos.y += cloud_ptr->points[i].y;
        n++;
      }
      centroid_pos.x /= n;
      centroid_pos.y /= n;

      ROS_DEBUG_STREAM("cluster centroid detected at x: " << centroid_pos.x << ", y: " << centroid_pos.y);

      center_estimate.x = centroid_pos.x * (1 + 2 * params_.mean_cone_radius 
                          / (M_PI * sqrtf(powf(centroid_pos.x, 2) + powf(centroid_pos.y, 2))));
      center_estimate.y = centroid_pos.y * (1 + 2 * params_.mean_cone_radius 
                          / (M_PI * sqrtf(powf(centroid_pos.x, 2) + powf(centroid_pos.y, 2))));
      //center_estimate.z = (CONE_Z_MAX + CONE_Z_MIN) / 2;
      center_estimate.z = 0;
      iter_ioint = center_estimate;

      ROS_DEBUG_STREAM("cluster centroid estimate detected at x: " << iter_ioint.x << ", y: " << iter_ioint.y);

      ROS_DEBUG("--------------------- Newton method iterations -------------------------");
      Eigen::Vector2f grad;
      int iter = 0;
      do
      {
        grad = calculateGradient(iter_ioint, indices, cloud_ptr);
        const auto hessian = calculateHessian(iter_ioint, indices, cloud_ptr);
        const auto delta = hessian * (-grad);
        iter_ioint.x += delta(0, 0);
        iter_ioint.y += delta(1, 0);
        ROS_DEBUG_STREAM("grad x: " << grad(0,0) << ", y: "<< grad(1,0));
        ROS_DEBUG_STREAM("hess (0,0): " << hessian(0,0) << ", 0,1): " << hessian(0,1) <<", (1,0): " 
                        << hessian(1,0) <<", (1,1): " << hessian(1,1));
        ROS_DEBUG_STREAM("delta x: " << delta(0,0) << ", y: " << delta(1,0));
      } while (grad.norm() > EPSILON_ERROR && ++iter < ITER_LIMIT);

      ROS_DEBUG_STREAM("processed cluster centroid x: " << iter_ioint.x << ", y: " << iter_ioint.y);

      sgtdv_msgs::Point2DStamped point;
      point.x = iter_ioint.x;
      point.y = iter_ioint.y;
      point.header.frame_id = "lidar";
      point.header.seq = i_n++;
      cones->points.push_back(point);
      ROS_DEBUG_STREAM("Pushed point into cones array");
    }
  }

  ROS_DEBUG_STREAM("At the end we have " << cones->points.size() << " cones");
}

Eigen::Vector2f LidarConeDetection::calculateGradient(const pcl::PointXYZ &point,
  const pcl::PointIndices &cluster_indices, const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud_ptr) const
{
  Eigen::Vector2f m = Eigen::Vector2f::Zero();

  for (const int i : cluster_indices.indices)
  {
    const float diffa = cloud_ptr->points[i].x - point.x;
    const float diffb = cloud_ptr->points[i].y - point.y;
    const float r = sqrtf(powf(diffa, 2) + powf(diffb, 2)); // length of radius vector for those alpha and beta values (see docs)
    m << diffa * (params_.mean_cone_radius / r - 1),
          diffb * (params_.mean_cone_radius / r - 1);

    /* auto diff = pcl::PointXYZ(point.x - cloud_ptr->points[i].x, point.y - cloud_ptr->points[i].y, 0);
    diffField.push_back(std::pair<pcl::PointXYZ, float>(diff, sqrtf(powf(diff.x, 2) + powf(diff.y, 2)))); */
  }

  m(0) *= 2;
  m(1) *= 2;

  return m;
}

Eigen::Matrix2f LidarConeDetection::calculateHessian(const pcl::PointXYZ &point, 
  const pcl::PointIndices &cluster_indices, const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_ptr) const
{
  Eigen::Matrix2f m = Eigen::Matrix2f::Zero();

  const int k = cluster_indices.indices.size(); // number of points in cluster
  float faa = 0.0f, fbb = 0.0f, fab = 0.0f;

  for(const int i : cluster_indices.indices)
  {
    float diffa = cloud_ptr->points[i].x - point.x;
    float diffb = cloud_ptr->points[i].y - point.y;
    float diffa2 = powf(diffa, 2);
    float diffb2 = powf(diffb, 2);
    float r = powf(sqrtf(diffa2 + diffb2), 3);

    faa += diffb2 / r;
    fbb += diffa2 / r;
    fab += (diffa * diffb) / r;
  }

  faa = 2 * (k - params_.mean_cone_radius * faa);
  fbb = 2 * (k - params_.mean_cone_radius * fbb);
  fab = 2 * params_.mean_cone_radius * fab;
  const float det = faa*fbb-fab*fab;

  m << fbb/det, -fab/det,
      -fab/det, faa/det;

  return m;
}
