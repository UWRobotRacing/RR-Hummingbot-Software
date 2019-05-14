/** @file lane_detection_processor.hpp
 *  @author Matthew Post
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */
#ifndef __LANE_DETECTION_HPP
#define __LANE_DETECTION_HPP

//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

//OPENCV includes
#include <opencv2/opencv.hpp>

//LOCAL

class LaneDetection
{
  public:
    LaneDetection(ros::NodeHandle nh);
    ~LaneDetection();
  private:
    void RGBCameraCallback(const sensor_msgs::Image& msg);
    ros::Subscriber test_subscriber;
    ros::Publisher test_publisher;
    ros::NodeHandle nh_;

    void InitializeSubscribers();
    void InitializePublishers();

};

#endif //__LANE_DETECTION_HPP