/**
 * @file rr_traffic_light.hpp
 * @brief Traffic Light Header File
 * @author Yuchi(Allan) Zhao
 * @author Waleed Ahmed (w29ahmed)
 * @competition IARRC 2019
 */

#ifndef TRAFFIC_LIGHT
#define TRAFFIC_LIGHT

// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>

// OpenCV includes
#include <opencv2/opencv.hpp>

class TrafficLightDetection
{
  public:
    TrafficLightDetection(ros::NodeHandle nh);
    void ImgCallback(const sensor_msgs::ImageConstPtr& msg);
    void SetBlobDetectorParams();
    void RedColorThreshold(const cv::Mat& input_img, cv::Mat& threshold_img);
    void RedLightDetection(const cv::Mat& threshold_img);
    void FindBoundRect(const cv::Mat& threshold_img, cv::Mat& crop_img, std::vector<cv::KeyPoint>& keypoints, const int max_area_index);

  private:
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_subscriber_;
    // image_transport::Publisher test_publisher_;

    // Blob detector
    cv::Rect boundRect_;
    cv::SimpleBlobDetector::Params params_;
    cv::Ptr<cv::SimpleBlobDetector> detector_;

    // Internal states/counters
    bool red_light_detected_;
    double default_pixel_ratio_;
    int red_light_counter_;
    int green_light_counter_;

    // Constants
    const double pixel_ratio_range_ = 0.40;
    const int frame_counter_max_ = 10;
};

#endif
