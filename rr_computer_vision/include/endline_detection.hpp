/** @file endline_detection.hpp
 * @author Angela Gu (angegu)
 * @author Toni Ogunmade (oluwatoni)
 * @author Waleed Ahmed (w29ahmed)
 * @author Yuchi(Allan) Zhao
 */

#ifndef ENDLINE_DETECTION_HPP
#define ENDLINE_DETECTION_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <std_srvs/Trigger.h>
// #include "msg_srv_names.hpp"

class EndlineCounter {
  public :
    EndlineCounter(ros::NodeHandle);
    void ImgCb(const sensor_msgs::ImageConstPtr&);
    static bool compareContourAreas( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 );
    
  private :
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber test_subscriber;
    image_transport::Publisher test_publisher;

    bool detection_status_;

    const int HighHue= 180;
    const int LowHue= 130;
    const int HighSat= 255;
    const int LowSat= 82;
    const int HighVal= 255;
    const int LowVal= 158;
};

#endif /*ENDLINE_DETECTION_HPP*/