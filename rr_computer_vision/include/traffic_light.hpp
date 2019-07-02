/**
 * @file rr_traffic_light.hpp
 * @brief Traffic Light Header File
 * @author Yuchi(Allan) Zhao
 * @author Waleed Ahmed
 * @competition IARRC 2019
 */

#ifndef TRAFFIC_LIGHT
#define TRAFFIC_LIGHT

// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>

// OPENCV includes
#include <opencv2/opencv.hpp>

class TrafficLightDetection
{
  public:
    TrafficLightDetection(ros::NodeHandle nh);
    void TrafficLightImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void SetParams();
    void RedLightDetection(const cv::Mat& threshold_img, int& maxAreaIndex, double& new_ratio);
    void ColorFilter(const cv::Mat& hsv_img, cv::Mat& threshold_img);
    void FindBoundRect(const int maxAreaIndex, cv::Mat& crop_img, const cv::Mat& threshold_img, std::vector<cv::KeyPoint>& keypoints);

  private:
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    image_transport::ImageTransport it_;
    // image_transport::Subscriber test_subscriber;
    // image_transport::Publisher test_publisher;
        
    cv::Rect boundRect_;
    cv::SimpleBlobDetector::Params params;
    cv::Ptr<cv::SimpleBlobDetector> detector_;
    bool red_light_detected_;
    double default_ratio_;
    int redLightCounter_;
    int greenLightCounter_;
};

#endif
