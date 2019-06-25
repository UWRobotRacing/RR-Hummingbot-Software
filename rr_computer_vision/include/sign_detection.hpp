/** @file sign_detection.hpp
 *  @author Martin Ethier (MartinEthier)
 *  @brief Sign Detection Class header file
 *  @competition IARRC 2019
 */
#ifndef SIGN_DETECTION_PROCESSOR_HPP
#define SIGN_DETECTION_PROCESSOR_HPP

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

// Synchronized Policy
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class SignDetection
{
  public:
    SignDetection(ros::NodeHandle nh);
    ~SignDetection();

  private:

    void InitializeSubscribers();
    void InitializePublishers();

    void RGBDepthSyncCameraCallback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& depth_msg);
    cv::Rect FilterSigns(std::vector<cv::Rect> signs, cv::Mat img);


    // Stuff for synchronized policy
    message_filters::Subscriber<sensor_msgs::Image> left_camera_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image> depth_map_subscriber_;   
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> RGB_DEPTH_POLICY;
    typedef message_filters::Synchronizer<RGB_DEPTH_POLICY> RGB_DEPTH_SYNC;
    boost::shared_ptr<RGB_DEPTH_SYNC> rgb_depth_sync_;

    ros::Subscriber left_header_subscriber_;
    ros::Subscriber depth_header_subscriber_;

    ros::Publisher test_publisher;
    ros::NodeHandle nh_;




    String cascade_name = "haarcascade_traffic_sign.xml";
    CascadeClassifier sign_cascade;


};

#endif //SIGN_DETECTION_PROCESSOR_HPP