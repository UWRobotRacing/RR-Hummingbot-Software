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
    void RGBCameraCallback(const sensor_msgs::Image& msg);
    void Multithreshold(const cv::Mat &input_image, cv::Mat &output_image);
    void FindWhite(const cv::Mat &input_image, cv::Mat &output_image);
    void get_BEV_image(const cv::Mat &img_bgr8_, cv::Mat &BEV_image_);
    void get_occupancy_grid(nav_msgs::OccupancyGrid &grid_msg_, const cv::Mat &out_);
    void process_image(const cv::Mat &img_bgr8_, cv::Mat &out_);

    ros::Subscriber zed_subscriber;
    ros::Publisher test_publisher;
    ros::Publisher pointList_pub_;
    ros::NodeHandle nh_;

    cv::Mat img_bgr8_;
    cv::Mat out_;
    

    cv_bridge::CvImagePtr cv_input_bridge_;
    cv_bridge::CvImage cv_output_bridge_;
    
    nav_msgs::OccupancyGrid grid_msg_;

    std::string point_vec_out_;

    

};

#endif //__LANE_DETECTION_HPP