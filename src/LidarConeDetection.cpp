/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Matej Dudák, Lukáš Lánik
/*****************************************************/


#include "../include/LidarConeDetection.h"

LidarConeDetection::LidarConeDetection() {
}

LidarConeDetection::~LidarConeDetection() {
}

void LidarConeDetection::Do(const sensor_msgs::PointCloud2::ConstPtr &msg) {
#ifdef SGT_DEBUG_STATE
    sgtdv_msgs::DebugState state;
    state.workingState = 1;
    m_visDebugPublisher.publish(state);
#endif

    // auto time = std::chrono::steady_clock::now();
    
    sgtdv_msgs::Point2DStampedArr coneArray;
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    pcl_conversions::toPCL(*msg, *cloud);
    pcl::PassThrough<pcl::PCLPointCloud2> passThrough;
    
    // ROS_INFO_STREAM("init time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-time).count());
    // time = std::chrono::steady_clock::now();
    passThrough.setInputCloud(cloudPtr);
    if (cloud->width > 0) {
        //filter data by intensity
        passThrough.setFilterFieldName("intensity");
        passThrough.setFilterLimits(CONE_INTENSITY_MIN, CONE_INTENSITY_MAX);
        passThrough.filter(*cloud);
    }
    if (cloud->width > 0) {
        //filter data by X axis (forward distance from lidar sensor)
        passThrough.setFilterFieldName("x");
        passThrough.setFilterLimits(CONE_X_MIN, CONE_X_MAX);
        passThrough.filter(*cloud);
    }
    if (cloud->width > 0) {
        //filter data by y axis (side distance from lidar sensor)
        passThrough.setFilterFieldName("y");
        passThrough.setFilterLimits(CONE_Y_MIN, CONE_Y_MAX);
        passThrough.filter(*cloud);
    }

    /*if (cloud->width > 0) {
        //filter data by z axis (vertical distance from lidar sensor)
        passThrough.setFilterFieldName("z");
        passThrough.setFilterLimits(CONE_Z_MIN, CONE_Z_MAX);
        passThrough.filter(*cloud);
    }*/

    // ROS_INFO_STREAM("filter time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-time).count());
    // time = std::chrono::steady_clock::now();

    if (cloud->width > 2) {
        /**
         * https://pointclouds.org/documentation/classpcl_1_1_euclidean_cluster_extraction.html
         * using Euclidean cluster extraction to get clusters of CONE_CLUSTER_POINTS within CONE_CLUSTER_RADIUS
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*cloud, *cloudFiltered);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        std::vector<pcl::PointIndices> clusterIndices;
        tree->setInputCloud(cloudFiltered);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(CONE_CLUSTER_RADIUS);
        ec.setMinClusterSize(CONE_CLUSTER_MIN_POINTS);
        ec.setMaxClusterSize(CONE_CLUSTER_MAX_POINTS);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloudFiltered);
        ec.extract(clusterIndices);

        /**
         * The closest point is selected from each cluster, mean cone radius is added to its bearing vector to 
         * compute center of the cone.
         */
        /*if (!clusterIndices.empty()) {
            coneArray.points.reserve(clusterIndices.size());
            int i_n = 0;
            for (const auto &indices: clusterIndices) {
                float x, y;
                float minDistance = std::numeric_limits<float>::max();
                for (int i: indices.indices) {
                    float distance = sqrt(pow(cloudFiltered->points[i].x, 2) + pow(cloudFiltered->points[i].y, 2) + pow(cloudFiltered->points[i].z, 2));
                    if (distance < minDistance) {
                        minDistance = distance;
                        x = cloudFiltered->points[i].x;
                        y = cloudFiltered->points[i].y;
                    }
                }
                const double alpha = atan(y / x);
                sgtdv_msgs::Point2DStamped point;
                point.x = x + cos(alpha) * CONE_RADIUS;
                point.y = y + sin(alpha) * CONE_RADIUS;
                point.header.frame_id = "lidar";
                point.header.seq = i_n++;
                point.header.stamp = msg->header.stamp;
                coneArray.points.emplace_back(point);
            }
        }*/

        /**
         * X -> select point with minimal y and take x from it
         * Y -> select point with minimal x and take y from it
         */
        /*
        if (!clusterIndices.empty()) {
            coneArray.points.reserve(clusterIndices.size());
            int i_n = 0;
            for (const auto &indices: clusterIndices) {
                float yMin = std::numeric_limits<float>::max();
                float xMin = std::numeric_limits<float>::max();
                sgtdv_msgs::Point2DStamped point;
                for (int i: indices.indices) {
                    if (yMin > cloudFiltered->points[i].y) {
                        yMin = cloudFiltered->points[i].y;
                        point.x = cloudFiltered->points[i].x;
                    }
                    if (xMin > cloudFiltered->points[i].x) {
                        xMin = cloudFiltered->points[i].x;
                        point.y = cloudFiltered->points[i].y;
                    }
                }
                point.header.frame_id = "lidar";
                point.header.seq = i_n++;
                point.header.stamp = msg->header.stamp;
                coneArray.points.push_back(point);
            }
        }*/

		/**
		 * Approximate centroid of visible semicricular arc of cone by taking average
		 * over all points in point cluser then scale the position vector of the centroid
		 * to get the approximate position of cone center
		 */
		
        if (!clusterIndices.empty()) {
            coneArray.points.reserve(clusterIndices.size());
            int i_n = 0;
            for (const auto &indices: clusterIndices) {
				pcl::PointXYZ centroidPos;
                for (int i: indices.indices) {
					centroidPos.x += cloudFiltered->points[i].x;
					centroidPos.y += cloudFiltered->points[i].y;
                }
				centroidPos.x /= indices.indices.size();
				centroidPos.y /= indices.indices.size();
				double centroidPosAbs = sqrt(pow(centroidPos.x, 2) + pow(centroidPos.y, 2));
				double scaleFactor = 1 + 2 * CONE_RADIUS / (M_PI * centroidPosAbs);
                sgtdv_msgs::Point2DStamped point;
				point.x = centroidPos.x * scaleFactor;
				point.y = centroidPos.y * scaleFactor;
                point.header.frame_id = "lidar";
                point.header.seq = i_n++;
                point.header.stamp = msg->header.stamp;
                coneArray.points.push_back(point);
    		}
		}

		/**
		 * Circular regression
		 */
        /*if (!clusterIndices.empty()) {
			coneArray.points.reserve(clusterIndices.size());
			int i_n = 0;
			for (const auto &indices: clusterIndices) {
				Eigen::Vector3f coneCenter(cloudFiltered->points[indices.indices.front()].x, cloudFiltered->points[indices.indices.front()].y, CONE_RADIUS);
				Eigen::Vector3f delta = deltaVec(indices, cloudFiltered, coneCenter);

				while (delta.norm() > EPSILON_ERROR) {
					coneCenter -= delta;
					delta = deltaVec(indices, cloudFiltered, coneCenter);
				}
				
				sgtdv_msgs::Point2DStamped point;
				point.x = coneCenter.x();
				point.y = coneCenter.y();
				point.header.frame_id = "lidar";
				point.header.seq = i_n++;
				point.header.stamp = msg->header.stamp;
				coneArray.points.push_back(point);
			}
		}*/
	}

    // ROS_INFO_STREAM("detection time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-time).count());
    // time = std::chrono::steady_clock::now();

    m_publisher.publish(coneArray);

#ifdef SGT_DEBUG_STATE
    state.numOfCones = coneArray.points.size();
    state.workingState = 0;
    m_visDebugPublisher.publish(state);
#endif
}

/*Eigen::Vector3f LidarConeDetection::deltaVec(pcl::PointIndices indices, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered, Eigen::Vector3f coneCenter) {
	int clusterSize = indices.indices.size();
	Eigen::VectorXf residual(clusterSize);
	Eigen::MatrixXf jaccobian(clusterSize, 3);
	Eigen::Vector3f delta;
	int j = 0;

	for (int i: indices.indices) {
		pcl::PointXYZ point = cloudFiltered->points[i];
		float xDiff = point.x - coneCenter.x();
		float yDiff = point.y - coneCenter.y();
		double rootDist = sqrt(pow(xDiff, 2) + pow(yDiff, 2));
		residual(j) = rootDist - CONE_RADIUS;
		jaccobian(j, 0) = -xDiff / rootDist;
		jaccobian(j, 1) = -yDiff / rootDist;
		jaccobian(j, 2) = -1.0;
		j++;
	}

	delta = (jaccobian.transpose() * jaccobian).inverse() * jaccobian.transpose() * residual;
	return delta;
}*/

void LidarConeDetection::SetPublisher(ros::Publisher publisher) {
    m_publisher = publisher;
}

