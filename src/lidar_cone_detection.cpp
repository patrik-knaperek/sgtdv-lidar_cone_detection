/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Matej Dudák, Lukáš Lánik
/*****************************************************/

/* ROS */
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>

/* Header */
#include "lidar_cone_detection.h"

sgtdv_msgs::Point2DStampedArr LidarConeDetection::update(const sensor_msgs::PointCloud2::ConstPtr &msg) const
{
  sgtdv_msgs::Point2DStampedArr cone_array;
  
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

    /**
     * Cone detection
     * Approximate centroid of visible semicricular arc of cone by taking average
     * over all points in point cluser then scale the position vector of the centroid
     * to get the approximate position of cone center
     */
  
    if(!cluster_indices.empty())
    {
      cone_array.points.reserve(cluster_indices.size());
      int i_n = 0;
      for(const auto &indices: cluster_indices)
      {
        pcl::PointXYZ centroid_pos;
        
        for (int i: indices.indices)
        {
          centroid_pos.x += cloud_filtered->points[i].x;
          centroid_pos.y += cloud_filtered->points[i].y;
        }

        centroid_pos.x /= indices.indices.size();
        centroid_pos.y /= indices.indices.size();
        const double centroid_pos_abs = sqrt(pow(centroid_pos.x, 2) + pow(centroid_pos.y, 2));
        const double scale_factor = 1 + 2 * params_.mean_cone_radius / (M_PI * centroid_pos_abs);
        
        sgtdv_msgs::Point2DStamped point;
        point.x = centroid_pos.x * scale_factor;
        point.y = centroid_pos.y * scale_factor;
        point.header.frame_id = "lidar";
        point.header.seq = i_n++;
        point.header.stamp = msg->header.stamp;
        cone_array.points.push_back(point);
      }
    }
  }
  return cone_array;
}
