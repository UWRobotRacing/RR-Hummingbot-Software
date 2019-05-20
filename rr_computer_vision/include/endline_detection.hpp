/** @file endline_detection.hpp
 * @author Angela Gu (angegu)  && Yuchi Zhao
 * @author Toni Ogunmade (oluwatoni)
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
  private :
    //ros::NodeHandle nh_;
    //image_transport::ImageTransport it_;
    
    ros::ServiceClient client_;
    bool detection_status_;
    int hysteresis_counter_;
    int hysteresis_constant_;
  
  public :
    EndlineCounter(ros::NodeHandle);

    image_transport::Subscriber test_subscriber;
    image_transport::Publisher test_publisher;

    void ImgCb(const sensor_msgs::ImageConstPtr&);
    bool BlobDetector(cv::Mat);
};

#endif /*ENDLINE_DETECTION_HPP*/