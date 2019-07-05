/** @file sign_detection.hpp
 *  @author Martin Ethier (MartinEthier)
 *  @brief Sign Detection Class header file
 *  @competition IARRC 2019
 */
#ifndef SIGN_DETECTION_PROCESSOR_HPP
#define SIGN_DETECTION_PROCESSOR_HPP

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>

// OpenCV
#include <opencv2/opencv.hpp>

// Other includes
#include <vector>

class SignDetection
{
  public:
    SignDetection(ros::NodeHandle);

  private:
    // Ros variables
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    //ros::Publisher sign_status_publisher_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_subscriber_;
    image_transport::Publisher img_publisher_;

    void RGBCameraCallback(const sensor_msgs::ImageConstPtr& left_msg);
    cv::Rect ExpandRect(cv::Rect rect_in);
    std::pair<cv::Mat, uint8_t> CheckArrowDir(cv::Mat sign);

    // Variables for tracking the sign through frames
    unsigned int consecutive_frames = 0;
    cv::Rect last_frame_bbox;

    // Haar cascade
    cv::CascadeClassifier sign_cascade;

    // Enum for publish message
    enum sign_status: uint8_t
    {
      NONE,
      LEFT,
      RIGHT,
      STRAIGHT
    };
};

#endif //SIGN_DETECTION_PROCESSOR_HPP