/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include <vector>
#include <sgtdv_msgs/Point2DArr.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "../../SGT_Macros.h"
#include <sgtdv_msgs/DebugState.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

//values for pcl constants are in centimeters
//constants for KNN clustering
#define CONE_CLUSTER_NEIGHBOURS 3
#define CONE_CLUSTER_RADIUS 0.2
#define CONE_INTENSITY_MIN 40
#define CONE_INTENSITY_MAX 250
#define CONE_X_MIN 0.75
#define CONE_X_MAX 30
#define CONE_Y_MIN -20
#define CONE_Y_MAX 20
#define CONE_Z_MIN 0.1
#define CONE_Z_MAX 0.3

//radius in meters from which 1 point will be selected from filtere data, representing cone
#define CONE_POINTS_RADIUS 1


class LidarConeDetection {
public:
    LidarConeDetection();

    ~LidarConeDetection();

    void SetPublisher(ros::Publisher publisher);

    void Do(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void VisualizeData(const sgtdv_msgs::Point2DArr &point2DArr);

#ifdef DEBUG_STATE

    void SetVisDebugPublisher(ros::Publisher publisher) { m_visDebugPublisher = publisher; }

    void SetFilteredPointsMarkerPublisher(ros::Publisher publisher) { m_filteredPointsMarkerPublisher = publisher; }

#endif

private:
    ros::Publisher m_publisher;

#ifdef DEBUG_STATE
    ros::Publisher m_visDebugPublisher;
    ros::Publisher m_filteredPointsMarkerPublisher;
#endif

};
