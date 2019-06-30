/** @file lane_detection_processor.hpp
 *  @author Waleed Ahmed (w29ahmed)
 *  @author Carson Chen
 *  @author Matthew Post
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2019
 */
#ifndef __LANE_DETECTION_HPP
#define __LANE_DETECTION_HPP

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>

// OpenCV includes
#include <opencv2/opencv.hpp>

class LaneDetection
{
  public:
    LaneDetection(ros::NodeHandle nh);
    ~LaneDetection();
    cv::Mat GetContours(const cv::Mat &image);
    
  private:
    ros::NodeHandle nh_;
    ros::Subscriber img_subscriber_;

    // Publishers for debugging
    ros::Publisher test_threshold_img_;
    ros::Publisher test_warp_img_;
    ros::Publisher grid_pub_;

    // Parameters
    cv::Mat warp_src_coords_;
    int min_contour_size_;

    void ImgCallback(const sensor_msgs::Image& msg);
    void InitializeSubscribers();
    void InitializePublishers();

    void Threshold(const cv::Mat &input_img, cv::Mat &output_img);
    void Warp(const cv::cuda::gpuMat &input_img, cv::cuda::gpuMat &output_img, const cv::Mat src);
    cv::Mat ContourFilter(const cv::Mat &img);
    void ConvertToOccupancyGrid(const cv::Mat &img, nav_msgs::OccupancyGrid &grid_msg);

};

#endif //__LANE_DETECTION_HPP