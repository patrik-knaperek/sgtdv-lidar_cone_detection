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
    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> radiusRemoval;

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

    if (cloud->width > 0) {
        //remove all data, that have less neighbour points than CONE_CLUSTER_NEIGHBOURS in within CONE_CLUSTER_RADIUS
        radiusRemoval.setInputCloud(cloudPtr);
        radiusRemoval.setRadiusSearch(CONE_CLUSTER_RADIUS);
        radiusRemoval.setMinNeighborsInRadius(CONE_CLUSTER_NEIGHBOURS);
        radiusRemoval.filter(*cloud);
    }

    //select only one point from points that are within 1m circle
    if (cloud->width > 2) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(*cloud, *cloud_xyz);

        std::vector<bool> pointSelected(cloud_xyz->size(), false);
        for (int i = 0; i < cloud_xyz->size(); i++) {
            for (int j = i + 1; j < cloud_xyz->size(); j++) {
                if (abs(cloud_xyz->points[i].x - cloud_xyz->points[j].x) < CONE_POINTS_RADIUS &&
                    abs(cloud_xyz->points[i].y - cloud_xyz->points[j].y) < CONE_POINTS_RADIUS &&
                    abs(cloud_xyz->points[i].z - cloud_xyz->points[j].z) < CONE_POINTS_RADIUS) {
                    pointSelected[i] = true;
                }
            }
        }

        int i_n = 0;
        for (int i = 0; i < pointSelected.size(); i++) {
            if (pointSelected[i]) {
                sgtdv_msgs::Point2D point;
                point.header.frame_id = "lidar";
                point.header.seq = i_n++;
                point.header.stamp = msg->header.stamp;
                point.x = cloud_xyz->points[i].x;
                point.y = cloud_xyz->points[i].y;
                coneArray->points.push_back(point);
            }
        }
    }
    coneArray->points.reserve(cloud->width);

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
