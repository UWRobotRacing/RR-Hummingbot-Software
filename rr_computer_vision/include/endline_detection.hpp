/** @file endline_detection.hpp
 *  @author Waleed Ahmed (w29ahmed)
 *  @author Yuchi(Allan) Zhao
 *  @competition IARRC 2019
 */

#ifndef ENDLINE_DETECTION_HPP
#define ENDLINE_DETECTION_HPP

// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>

// OpenCV includes
#include <opencv2/opencv.hpp>

class EndlineDetection
{
  public :
    EndlineDetection(ros::NodeHandle);

  private :
    // Ros variables
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_subscriber_;
    // image_transport::Publisher test_publisher_;

    // Callback and lambda function for detecting the endline
    void EndlineImgCallback(const sensor_msgs::ImageConstPtr&);
    static bool compareContourAreas( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 );

    // Internal status variables
    bool detection_status_;
    int endline_counter_;

    // 1500 was used as the max contour threshold as that is what the area of the contour
    // is when the endline first comes into view (approximately)
    const double contour_area_cutoff_ = 3000.00;

    cv::Scalar hsv_lower_bounds_;
    cv::Scalar hsv_upper_bounds_;
};

#endif /*ENDLINE_DETECTION_HPP*/