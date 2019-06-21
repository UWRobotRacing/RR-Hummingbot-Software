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

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>

//LOCAL

class LaneDetection
{
  public:
    LaneDetection(ros::NodeHandle nh);
    ~LaneDetection();
    cv::Mat GetContours(const cv::Mat &image);
    
  private:
    void InitializeSubscribers();
    void InitializePublishers();
    void LeftCameraCallback(const sensor_msgs::Image& msg);
    void RightCameraCallback(const sensor_msgs::Image& msg);
    void Multithreshold(const cv::Mat &input_image, cv::Mat &output_image);
    void FindWhite(const cv::Mat &input_image, cv::Mat &output_image);
    void get_BEV_image(const cv::Mat &img_bgr8_, cv::Mat &BEV_image_, const cv::Mat src);
    void get_occupancy_grid(nav_msgs::OccupancyGrid &grid_msg_, const cv::Mat &out_);
    void process_image(const cv::Mat &img_bgr8_, cv::Mat &out_, int left_or_right);

    ros::Subscriber left_subscriber;
    ros::Subscriber right_subscriber;

    // Publishers for debugging
    ros::Publisher test_left_warp_img;
    ros::Publisher test_right_warp_img;
    ros::Publisher test_left_warp_threshold_img;
    ros::Publisher test_right_warp_threshold_img;

    ros::Publisher left_pub_;
    ros::Publisher right_pub_;
    ros::NodeHandle nh_;

    cv::Mat src;
    cv::Mat left_img_bgr8_;
    cv::Mat left_out_;
    cv::Mat right_img_bgr8_;
    cv::Mat right_out_;
    
    nav_msgs::OccupancyGrid left_grid_msg_;
    nav_msgs::OccupancyGrid right_grid_msg_;

};

#endif //__LANE_DETECTION_HPP