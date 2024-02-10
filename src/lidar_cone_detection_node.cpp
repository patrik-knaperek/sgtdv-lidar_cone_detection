/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Matej Dudák
/*****************************************************/


#include "../include/lidar_cone_detection_synch.h"

int main(int argc, char **argv) {
    LidarConeDetectionSynch synchObj;

    ros::init(argc, argv, "lidarConeDetection");
    ros::NodeHandle handle;
    ros::Publisher publisher = handle.advertise<sgtdv_msgs::Point2DStampedArr>("lidar_cones", 1);

#ifdef SGT_DEBUG_STATE
    ros::Publisher lidarConeDetectionDebugStatePublisher = handle.advertise<sgtdv_msgs::DebugState>("lidar_cone_detection_debug_state", 1);
    synchObj.SetVisDebugPublisher(lidarConeDetectionDebugStatePublisher);
#endif

    synchObj.SetPublisher(publisher);

    ros::Subscriber cameraSub = handle.subscribe("camera_ready", 1, &LidarConeDetectionSynch::ReceiveSignal, &synchObj);
    ros::Subscriber pclSub = handle.subscribe("velodyne_points", 1, &LidarConeDetectionSynch::Do, &synchObj);

    ros::spin();

    return 0;
}