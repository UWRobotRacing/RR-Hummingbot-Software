/** @file computer_vision.hpp
 *  @author Andrew Jin (D29Jin)
 *  @competition IARRC 2019
 */
#ifndef COMPUTER_VISION
#define COMPUTER_VISION


//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

//OPENCV includes
#include <opencv2/opencv.hpp>

//LOCAL

class ComputerVision
{
  public:
    ComputerVision(ros::NodeHandle nh);
    ~ComputerVision();
    void InitializeSubscribers();
  private:
    //void RGBCameraCallback(const sensor_msgs::Image& msg);
    void LeftRightSyncCameraCallback(const sensor_msgs::Image& left_msg, const sensor_msgs::Image& right_msg);
    void LeftCameraCallback(const sensor_msgs::Image& msg);
    void RightCameraCallback(const sensor_msgs::Image& msg);
    void DepthCameraCallback(const sensor_msgs::Image& msg);
    ros::Subscriber rgb_camera_subscriber_;

    message_filters::Subscriber<sensor_msgs::Image> left_camera_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image> right_camera_subscriber_;
    TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync;

    //ros::Subscriber left_camera_subscriber_;
    //ros::Subscriber right_camera_subscriber_;
    ros::Subscriber depth_camera_subscriber_;
    ros::NodeHandle nh_;
};

#endif // COMPUTER_VISION