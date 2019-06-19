/**
 * @file rr_traffic_light.hpp
 * @brief Traffic Light Header File
 * @author Jamie Kim
 * @author Jason Leung
 * @author Toni Ogunmade(oluwatoni)
 * @author Adrian Malaran
 * @competition IARRC 2018
 */

#ifndef RR_TRAFFIC_LIGHT
#define RR_TRAFFIC_LIGHT

// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

// OPENCV includes
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>


class TrafficLightProcessor
{
    private:
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber test_subscriber;
    image_transport::Publisher test_publisher;
        
    bool red_light_detected;
    int red_Pixel_Counter;
    double default_ratio;

    cv::Rect boundRect;
    cv::Mat rectSection;
    public:
    TrafficLightProcessor(ros::NodeHandle nh);
    void TrafficLightImageCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif
