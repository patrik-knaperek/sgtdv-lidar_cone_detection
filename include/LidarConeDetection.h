/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include <vector>
#include <sgtdv_msgs/Point2DArr.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui_c.h>

class LidarConeDetection
{
public:
    LidarConeDetection();
    ~LidarConeDetection();

    void SetPublisher(ros::Publisher publisher);
    void Do(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void VisualizeData(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
private:
    ros::Publisher m_publisher;
    cv::Mat m_image;
    int m_spaceDepth;
    int m_spaceWidth;
    int m_spaceToImgRatio;
    int m_imgWidth;
    int m_imgHeight;
    int m_pointRadius;
    cv::Point2i m_imgOrigin;
    std::vector<cv::Vec3f> m_coneCoords;

    void DrawPointToImage(const cv::Point2d &spacePoint);
    cv::Point2d getSpaceCoords(const cv::Point2i &imgPoint) const;
    cv::Point2i getImgCoords(const cv::Point2d &spacePoint) const;
    bool IsImgPointInImg(const cv::Point2i &imgPoint) const;
};
