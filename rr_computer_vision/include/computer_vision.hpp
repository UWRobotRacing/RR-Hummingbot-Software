/** @file computer_vision.hpp
 *  @author Andrew Jin (D29Jin)
 *  @competition IARRC 2019
 */
#ifndef COMPUTER_VISION
#define COMPUTER_VISION


//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//OPENCV includes
#include <opencv2/opencv.hpp>

//LOCAL

class ComputerVision
{
  public:
    ComputerVision(ros::NodeHandle nh);
    ~ComputerVision();

  private:
    void InitializeSubscribers();
    void InitializePublishers();

    void RGBCameraCallback(const sensor_msgs::Image& msg);
    void LeftRightSyncCameraCallback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg);
    void LeftCameraCallback(const sensor_msgs::Image& msg);
    void RightCameraCallback(const sensor_msgs::Image& msg);
    void DepthCameraCallback(const sensor_msgs::Image& msg);
    ros::Subscriber rgb_camera_subscriber_;

    // Stuff for synchronized policy
    message_filters::Subscriber<sensor_msgs::Image> left_camera_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image> right_camera_subscriber_;    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> LRPOLICY;
    typedef message_filters::Synchronizer<LRPOLICY> LRSYNC;
    boost::shared_ptr<LRSYNC> lrsync_;

    ros::Subscriber left_header_subscriber_;
    ros::Subscriber right_header_subscriber_;

    ros::Subscriber depth_camera_subscriber_;
    ros::Publisher test_publisher;
    ros::NodeHandle nh_;
};

#endif // COMPUTER_VISION