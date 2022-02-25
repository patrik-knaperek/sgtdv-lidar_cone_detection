/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include "LidarConeDetection.h"
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include "../../SGT_Macros.h"

class LidarConeDetectionSynch
{
public:
    LidarConeDetectionSynch();
    ~LidarConeDetectionSynch();

    void ReceiveSignal(const std_msgs::Empty::ConstPtr &msg);
    void Do(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void SetPublisher(ros::Publisher publisher);

#ifdef DEBUG_STATE
    void SetVisDebugPublisher(ros::Publisher publisher) { m_lidarConeDetection.SetVisDebugPublisher(publisher); }
    void SetFilteredPointsMarkerPublisher(ros::Publisher publisher) { m_lidarConeDetection.SetFilteredPointsMarkerPublisher(publisher); }
#endif

private:    
    LidarConeDetection m_lidarConeDetection;
    bool m_signalReceived;
};