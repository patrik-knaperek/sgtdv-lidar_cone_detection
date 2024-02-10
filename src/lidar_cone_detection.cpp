/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Matej Dudák, Lukáš Lánik
/*****************************************************/

#include "../include/lidar_cone_detection.h"

LidarConeDetection::LidarConeDetection(ros::NodeHandle& nh) :
    /* ROS interface init */
    publisher_(nh.advertise<sgtdv_msgs::Point2DStampedArr>("lidar_cones", 1)),
    pcl_sub_(nh.subscribe("velodyne_points", 1, &LidarConeDetection::lidarCallback, this))
#ifdef SGT_DEBUG_STATE
    , vis_debug_publisher_(nh.advertise<sgtdv_msgs::DebugState>("lidar_cone_detection_debug_state", 2))
#endif
{
}

void LidarConeDetection::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) const{
#ifdef SGT_DEBUG_STATE
    sgtdv_msgs::DebugState state;
    state.stamp = ros::Time::now();
    state.working_state = 1;
    vis_debug_publisher_.publish(state);
#endif

    sgtdv_msgs::Point2DStampedArr cone_array;
    
    /* Pointcloud filtering */
    
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloud_ptr(cloud);

    pcl_conversions::toPCL(*msg, *cloud);
    pcl::PassThrough<pcl::PCLPointCloud2> pass_through;
    
    pass_through.setInputCloud(cloud_ptr);
    if (cloud->width > 0) {
        //filter data by intensity
        pass_through.setFilterFieldName("intensity");
        pass_through.setFilterLimits(CONE_INTENSITY_MIN, CONE_INTENSITY_MAX);
        pass_through.filter(*cloud);
    }
    if (cloud->width > 0) {
        //filter data by X axis (forward distance from lidar sensor)
        pass_through.setFilterFieldName("x");
        pass_through.setFilterLimits(CONE_X_MIN, CONE_X_MAX);
        pass_through.filter(*cloud);
    }
    if (cloud->width > 0) {
        //filter data by y axis (side distance from lidar sensor)
        pass_through.setFilterFieldName("y");
        pass_through.setFilterLimits(CONE_Y_MIN, CONE_Y_MAX);
        pass_through.filter(*cloud);
    }

    if (cloud->width > 0) {
        //filter data by z axis (vertical distance from lidar sensor)
        pass_through.setFilterFieldName("z");
        pass_through.setFilterLimits(CONE_Z_MIN, CONE_Z_MAX);
        pass_through.filter(*cloud);
    }

    if (cloud->width > 2) {
        /**
         * Pointcloud clustering
         * https://pointclouds.org/documentation/classpcl_1_1_euclidean_cluster_extraction.html
         * using Euclidean cluster extraction to get clusters of CONE_CLUSTER_POINTS within CONE_CLUSTER_RADIUS
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*cloud, *cloud_filtered);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        std::vector<pcl::PointIndices> cluster_indices;
        tree->setInputCloud(cloud_filtered);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(CONE_CLUSTER_RADIUS);
        ec.setMinClusterSize(CONE_CLUSTER_MIN_POINTS);
        ec.setMaxClusterSize(CONE_CLUSTER_MAX_POINTS);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

		/**
		 * Cone detection
         * Approximate centroid of visible semicricular arc of cone by taking average
		 * over all points in point cluser then scale the position vector of the centroid
		 * to get the approximate position of cone center
		 */
		
        if (!cluster_indices.empty()) {
            cone_array.points.reserve(cluster_indices.size());
            int i_n = 0;
            for (const auto &indices: cluster_indices) {
				pcl::PointXYZ centroid_pos;
                for (int i: indices.indices) {
					centroid_pos.x += cloud_filtered->points[i].x;
					centroid_pos.y += cloud_filtered->points[i].y;
                }
				centroid_pos.x /= indices.indices.size();
				centroid_pos.y /= indices.indices.size();
				double centroid_pos_abs = sqrt(pow(centroid_pos.x, 2) + pow(centroid_pos.y, 2));
				double scale_factor = 1 + 2 * CONE_RADIUS / (M_PI * centroid_pos_abs);
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

    publisher_.publish(cone_array);

#ifdef SGT_DEBUG_STATE
    state.num_of_cones = cone_array.points.size();
    state.stamp = ros::Time::now();
    state.working_state = 0;
    vis_debug_publisher_.publish(state);
#endif
}
