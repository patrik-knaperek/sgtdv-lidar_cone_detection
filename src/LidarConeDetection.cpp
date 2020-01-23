#include "../include/LidarConeDetection.h"

LidarConeDetection::LidarConeDetection()
{
    m_spaceToImgRatio = 100;
    m_spaceDepth = 8;
    m_spaceWidth = 9;
    m_imgWidth = m_spaceWidth * m_spaceToImgRatio;
    m_imgHeight = m_spaceDepth * m_spaceToImgRatio;
    m_pointRadius = 3;

    m_image = cv::Mat(m_imgHeight, m_imgWidth, CV_8UC1);
    m_imgOrigin = cv::Point2i(m_imgWidth / 2., m_imgHeight);
}

LidarConeDetection::~LidarConeDetection()
{
    
}

void LidarConeDetection::Do(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    sgtdv_msgs::Point2DArrPtr coneCoordsMsg(new sgtdv_msgs::Point2DArr);
    float const *temp;

    m_image = cv::Scalar(255, 255, 255);
    m_coneCoords.clear();

    m_coneCoords.reserve(20);
    coneCoordsMsg->points.reserve(20);

    for (int i = 0; i < msg->width; i++)
    {
        temp = reinterpret_cast<const float*>(&msg->data[i*msg->point_step]);
        geometry_msgs::Point point;

        point.x = *temp;
        point.y = *(temp + 1);
        point.z = *(temp + 2);

        if (point.z > 0.1)
        {
            DrawPointToImage(cv::Point2d(point.y, point.x));
        }
    }

    HoughCircles(m_image, m_coneCoords, CV_HOUGH_GRADIENT, 2, 50., 255, 15, 5, 15);

    for(size_t i = 0; i < m_coneCoords.size(); i++)
    {
        sgtdv_msgs::Point2D point;
        cv::Point2d dPoint = getSpaceCoords(cv::Point2i(m_coneCoords[i][0], m_coneCoords[i][1]));
        point.x = dPoint.y;
        point.y = dPoint.x;

        coneCoordsMsg->points.push_back(point);
    }

    m_publisher.publish(coneCoordsMsg);
}

void LidarConeDetection::SetPublisher(ros::Publisher publisher)
{
    m_publisher = publisher;
}

cv::Point2i LidarConeDetection::getImgCoords(const cv::Point2d &spacePoint) const{
    int imgX, imgY;

    imgX = m_imgOrigin.x - spacePoint.x * m_spaceToImgRatio;
    imgY = m_imgOrigin.y - spacePoint.y * m_spaceToImgRatio;

    return cv::Point2i(imgX, imgY);
}

cv::Point2d LidarConeDetection::getSpaceCoords(const cv::Point2i &imgPoint) const{
    double spaceX, spaceY;

    spaceX = -(imgPoint.x - m_imgOrigin.x) / static_cast<double>(m_spaceToImgRatio);
    spaceY = (m_imgOrigin.y - imgPoint.y) / static_cast<double>(m_spaceToImgRatio);

    return cv::Point2d(spaceX, spaceY);
}

bool LidarConeDetection::IsImgPointInImg(const cv::Point2i &imgPoint) const{
    if(imgPoint.x < 0 || imgPoint.x > m_imgWidth){
        return false;
    }

    if(imgPoint.y < 0 || imgPoint.y > m_imgHeight){
        return false;
    }

    return true;
}

void LidarConeDetection::DrawPointToImage(const cv::Point2d &spacePoint){
    cv::Point2i pixel = getImgCoords(spacePoint);

    if(IsImgPointInImg(pixel)){
        circle(m_image, pixel, m_pointRadius, cv::Scalar(0,255,255), CV_FILLED, 8, 0);
    }
    else{
        return;
    }
}
