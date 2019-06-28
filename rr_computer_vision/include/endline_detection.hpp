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

    // 1500 was used as the max contour threshold as that is what the area of the contour
    // is when the endline first comes into view (approximately)
    const double contour_area_cutoff_ = 1500.00;

    // Colour thresholding parameters
    const int low_hue_ = 130;
    const int high_hue_ = 180;

    // Outdoor sunny facing away from sun or cloudy: [62, 255]
    // Outdoor sunny facing sun: [0, 255]
    // Indoor night time: [82, 255]
    const int low_sat_ = 62;
    const int high_sat_ = 255;

    // Outdoor sunny facing away from sun or cloudy: [55, 255]
    // Outdoor sunny facing sun: [41, 177]
    // Indoor night time: [158, 255]
    const int low_val_ = 55;
    const int high_val_ = 255;
};

#endif /*ENDLINE_DETECTION_HPP*/