/** @file endline_detection.hpp
 * @author Waleed Ahmed (w29ahmed)
 * @author Yuchi(Allan) Zhao
 */

#ifndef ENDLINE_DETECTION_HPP
#define ENDLINE_DETECTION_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
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
    // image_transport::Publisher img_publisher_;

    // Callback and lambda function for detecting the endline
    void EndlineImgCallback(const sensor_msgs::ImageConstPtr&);
    static bool compareContourAreas( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 );

    // Internal status variables
    bool detection_status_;
    int endline_counter_;

    // Colour thresholding parameters
    const int high_hue_ = 180;
    const int low_hue_ = 130;
    const int high_sat_ = 255;
    const int low_sat_ = 82;
    const int high_val_ = 255;
    const int low_val_ = 158;
};

#endif /*ENDLINE_DETECTION_HPP*/