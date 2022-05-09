/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Matej Dudák
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

    sgtdv_msgs::Point2DArrPtr coneArray(new sgtdv_msgs::Point2DArr);
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    pcl_conversions::toPCL(*msg, *cloud);
    pcl::PassThrough<pcl::PCLPointCloud2> passThrough;

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

    if (cloud->width > 0) {
        //filter data by z axis (vertical distance from lidar sensor)
        passThrough.setFilterFieldName("z");
        passThrough.setFilterLimits(CONE_Z_MIN, CONE_Z_MAX);
        passThrough.filter(*cloud);
    }

    if (cloud->width > 2) {
        /**
         * https://pointclouds.org/documentation/classpcl_1_1_euclidean_cluster_extraction.html
         * using Euclidean cluster extraction to get clusters of CONE_CLUSTER_POINTS within CONE_CLUSTER_RADIUS
         *
         * in for loop, closest point is selected from each cluster
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

        if (!clusterIndices.empty()) {
            coneArray->points.reserve(clusterIndices.size());
            int i_n = 0;
            for (const auto &indices: clusterIndices) {
                sgtdv_msgs::Point2D point;
                point.header.frame_id = "velodyne";
                point.header.seq = i_n++;
                point.header.stamp = msg->header.stamp;
                float minDistance = std::numeric_limits<float>::max();
                for (int i: indices.indices) {
                    float distance = sqrt(pow(cloudFiltered->points[i].x, 2) + pow(cloudFiltered->points[i].y, 2));
                    if (distance < minDistance) {
                        minDistance = distance;
                        point.x = cloudFiltered->points[i].x;
                        point.y = cloudFiltered->points[i].y;
                    }
                }
                coneArray->points.push_back(point);
            }
        }
    }

    m_publisher.publish(coneArray);

#ifdef SGT_DEBUG_STATE
    state.numOfCones = coneArray->points.size();
    state.workingState = 0;
    m_visDebugPublisher.publish(state);
#endif

}

void LidarConeDetection::SetPublisher(ros::Publisher publisher) {
    m_publisher = publisher;
}
