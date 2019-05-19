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
    void Multithreshold(const cv::Mat &input_image, const cv::Mat &bounds, cv::Mat &output_image);
    void FindWhite(const cv::Mat &input_image, const cv::Scalar bounds, int patch_size, cv::Mat &output_image);
    cv::Mat GetContours(const cv::Mat &image, int min_size);
    
  private:
    void InitializeSubscribers();
    void InitializePublishers();
    void RGBCameraCallback(const sensor_msgs::Image& msg);

    int adapt_hsv_patch_size_;
    int blob_size_;

    ros::Subscriber zed_subscriber;
    ros::Publisher test_publisher;
    ros::Publisher pointList_pub_;
    ros::NodeHandle nh_;

    cv::Mat im_input_;
    cv::Mat Im1_HSV_;
    cv::Mat Im1_HSV_warped_;
    cv::Mat transform_matrix_;
    cv::Mat multibounds_;
    cv::Mat mask_warped_1_;
    cv::Mat mask_warped_2_;
    cv::Mat mask_;
    cv::Mat out;
    cv::Mat src;
    cv::Mat dst;
    cv::Mat M;
    cv::Mat BEV_image;
    cv::Mat img_gray;

    cv::Size BEV_size_;
    cv::Scalar bounds_;
    cv_bridge::CvImagePtr cv_input_bridge_;
    cv_bridge::CvImage cv_output_bridge_;
    
    nav_msgs::MapMetaData meta_data_;
    nav_msgs::OccupancyGrid grid_msg_;

    std::string point_vec_out_;

    uchar* data_pointer_;
    uchar value1_;


    std::vector<int8_t> occupancy_;

};

#endif //__LANE_DETECTION_HPP