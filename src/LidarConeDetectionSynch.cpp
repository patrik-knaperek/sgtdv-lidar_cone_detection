#include "../include/LidarConeDetectionSynch.h"

LidarConeDetectionSynch::LidarConeDetectionSynch()
{
    m_signalReceived = false;
}

LidarConeDetectionSynch::~LidarConeDetectionSynch()
{

}

void LidarConeDetectionSynch::ReceiveSignal(const std_msgs::Empty::ConstPtr &msg)
{
    m_signalReceived = true;
}

void LidarConeDetectionSynch::Do(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if(m_signalReceived)
    {
        m_signalReceived = false;
        m_lidarConeDetection.Do(msg);
    }
}

void LidarConeDetectionSynch::SetPublisher(ros::Publisher publisher)
{
    m_lidarConeDetection.SetPublisher(publisher);
}
