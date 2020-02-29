/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include "../include/LidarConeDetectionSynch.h"
#include <std_msgs/Empty.h>
#include <sgtdv_msgs/Point2DArr.h>
#include <sensor_msgs/PointCloud2.h>

int main (int argc, char** argv)
{
    LidarConeDetectionSynch synchObj;

    ros::init(argc, argv, "lidarConeDetection");
    ros::NodeHandle handle;
    ros::Publisher publisher = handle.advertise<sgtdv_msgs::Point2DArr>("lidar_cones", 1);

    synchObj.SetPublisher(publisher);    

    ros::Subscriber cameraSub = handle.subscribe("camera_ready", 1, &LidarConeDetectionSynch::ReceiveSignal, &synchObj);
    ros::Subscriber pclSub = handle.subscribe("velodyne_points", 1, &LidarConeDetectionSynch::Do, &synchObj);
    
    ros::spin();

    return 0;
}