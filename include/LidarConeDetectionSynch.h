#include <ros/ros.h>
#include "LidarConeDetection.h"
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>

class LidarConeDetectionSynch
{
public:
    LidarConeDetectionSynch();
    ~LidarConeDetectionSynch();

    void ReceiveSignal(const std_msgs::Empty::ConstPtr &msg);
    void Do(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void SetPublisher(ros::Publisher publisher);

private:    
    LidarConeDetection m_lidarConeDetection;
    bool m_signalReceived;
};