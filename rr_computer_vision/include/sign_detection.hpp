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
#include <std_msgs/Bool.h>
#include <rr_computer_vision/TrafficSign.h>

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
    ros::Subscriber horiz_lane_monitor_sub_;
    ros::Publisher sign_status_pub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber zed_right_img_sub_;
    // image_transport::Publisher img_publisher_;

    void InitializeSubscribers();
    void InitializePublishers();

    void HorizontalLaneMonitorCallback(const std_msgs::Bool& lane_crossed_msg);
    void ZedCameraImgCallback(const sensor_msgs::ImageConstPtr& right_img_msg);
    uint8_t CheckArrowDir(const cv::Mat& sign);

    // Variable for msg from horizontal lane monitor
    bool horizontal_lane_crossed;

    // Variables for tracking the sign through frames
    unsigned int consecutive_frames = 0;
    cv::Rect last_frame_bbox;
    bool check_arrow = false;

    // Array to accumulate direction frequency when tracking arrow direction
    std::vector<uint16_t> arrow_status_accumulator = std::vector<uint16_t>(4);

    // Haar cascade
    cv::CascadeClassifier sign_cascade;
};

#endif //SIGN_DETECTION_PROCESSOR_HPP