/** @file lane_detection.cpp
 *  @author Andrew Jin
 *  @competition IARRC 2019
 */

#include "lane_detection.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <vector>
using namespace std;
/*
 * @name LaneDetection 
 * @brief Constructor
 * @param nh: ROS node handler
*/
LaneDetection::LaneDetection(ros::NodeHandle nh) {
  nh_ = nh;
  // Initialize publishers and subscribers
  InitializeSubscribers();
  InitializePublishers();
}

/*
 * @name ~LaneDetection
 * @brief Destructor
*/
LaneDetection::~LaneDetection() {

}

void LaneDetection::InitializeSubscribers() {
  left_subscriber = nh_.subscribe("/zed/rgb/image_rect_color", 1, &LaneDetection::LeftCameraCallback, this);
  right_subscriber = nh_.subscribe("/zed/rgb/image_rect_color", 1, &LaneDetection::RightCameraCallback, this);
}

void LaneDetection::InitializePublishers() {  
  // Setup debug rostopics
  test_left_warp_img = nh_.advertise<sensor_msgs::Image>("/test_left_warp_img", 1);
  test_right_warp_img = nh_.advertise<sensor_msgs::Image>("/test_right_warp_img", 1);
  test_left_warp_threshold_img = nh_.advertise<sensor_msgs::Image>("/test_left_warp_threshold_img", 1);
  test_right_warp_threshold_img = nh_.advertise<sensor_msgs::Image>("/test_right_warp_threshold_img", 1);

  left_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/output_point_list_left_cam", 1);
  right_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/output_point_list_right_cam", 1);
}

void LaneDetection::LeftCameraCallback(const sensor_msgs::Image& msg){

  // Convert sensor_msgs::Image to a BGR8 cv::Mat
  cv_bridge::CvImagePtr cv_bridge_bgr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  left_img_bgr8_ = cv_bridge_bgr->image;

  // Process input image
  process_image(left_img_bgr8_, left_out_, 0);
  
  // Convert to ros message
  cv_bridge::CvImage img_bridge_output;
  std_msgs::Header header; // empty header
  header.stamp = ros::Time::now(); // time
  img_bridge_output = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, left_out_);
  
  // Publish image
  test_left_warp_threshold_img.publish(img_bridge_output.toImageMsg());

  // Calculate occupancy grid
  get_occupancy_grid(left_grid_msg_, left_out_);
  // publish occupancy grid
  left_pub_.publish(left_grid_msg_);
}

void LaneDetection::RightCameraCallback(const sensor_msgs::Image& msg){
  // Convert sensor_msgs::Image to a BGR8 cv::Mat
  cv_bridge::CvImagePtr cv_bridge_bgr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  right_img_bgr8_ = cv_bridge_bgr->image;

  // Process input image
  process_image(right_img_bgr8_, right_out_, 1);
  
  // Convert to ros message
  cv_bridge::CvImage img_bridge_output;
  std_msgs::Header header; // empty header
  header.stamp = ros::Time::now(); // time
  img_bridge_output = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, right_out_);
  
  // Publish image
  test_right_warp_threshold_img.publish(img_bridge_output.toImageMsg());

  // Calculate occupancy grid
  get_occupancy_grid(right_grid_msg_, right_out_);
  // publish occupancy grid
  right_pub_.publish(right_grid_msg_);
}

void LaneDetection::get_occupancy_grid(nav_msgs::OccupancyGrid &grid_msg_, const cv::Mat &out_){
  uchar* data_pointer_;
  uchar value1_;
  std::vector<int8_t> occupancy_;
  nav_msgs::MapMetaData meta_data_;
  occupancy_.clear();
  occupancy_.reserve(out_.cols * out_.rows);

  data_pointer_ = out_.data;
  for (int i = 0; i < (out_.rows * out_.cols); i++)
  {
    value1_ = *data_pointer_;
    data_pointer_++;
    if (value1_ == 0)
    {
      occupancy_.push_back(-1);
    }
    else
    {
      occupancy_.push_back(100);
    }
  }
  grid_msg_.data = occupancy_;
  meta_data_.height = 720;
  meta_data_.width = 1280;
  meta_data_.resolution = 0.01;
  meta_data_.origin.position.x = 0;
  meta_data_.origin.position.y = 0;

  grid_msg_.info = meta_data_;
}

void LaneDetection::get_BEV_image(const cv::Mat &img_bgr8_, cv::Mat &BEV_image_, const cv::Mat src){
  cv::Mat dst = (cv::Mat_<float>(4,2) << 300.0, 0, 900.0, 0, 900.0, 710.0, 300.0, 710.0);
  cv::Mat M_ = cv::getPerspectiveTransform(src, dst);
  cv::warpPerspective(img_bgr8_,BEV_image_,M_,img_bgr8_.size());
}

void LaneDetection::Multithreshold(const cv::Mat &input_image, cv::Mat &output_image) {
  cv::cvtColor(input_image, input_image, CV_BGR2HSV, 3);

  cv::Mat bounds_ = (cv::Mat_<double>(3,6) << 0, 100, 140, 120, 255, 255, 0, 0, 250, 255, 25, 255, 25, 5, 186, 130, 50, 255);
  //Confirm Dims and define binary mask_
  cv::Mat mask_(input_image.rows, input_image.cols, CV_8U, cv::Scalar::all(0));
  mask_.copyTo(output_image);

  // define min/max for proper wrapping
  double max = DBL_MAX;
  double min = -DBL_MAX;

  // loop through threshold bands, seperate into lower & upper thresholds, if any channel has lower > upper,
  // split into 2 bands, else keep original. 
  for (int i = 0; i < bounds_.rows; i++)
  {
    cv::Mat lowerbound(bounds_.colRange(0, bounds_.cols / 2).rowRange(i, i + 1));
    cv::Mat upperbound(bounds_.colRange(bounds_.cols / 2, bounds_.cols).rowRange(i, i + 1));
    cv::Mat flipedthresh = lowerbound > upperbound;

    if (sum(flipedthresh)[0] > 0)
    {
      cv::Mat lowerbound2(lowerbound.clone());
      cv::Mat upperbound2(upperbound.clone());

      // use logical array indexing based on flipedthresh
      lowerbound2.setTo(min, flipedthresh);
      upperbound2.setTo(max, flipedthresh);

      cv::inRange(input_image, lowerbound, upperbound2, mask_);
      output_image = output_image | mask_;
      cv::inRange(input_image, lowerbound2, upperbound, mask_);
      output_image = output_image | mask_;
    } 
    else 
    {
      cv::inRange(input_image, lowerbound, upperbound, mask_);
      output_image = output_image | mask_;
    }
  }
}

void LaneDetection::FindWhite(const cv::Mat &input_image, cv::Mat &output_image) {
  cv::Scalar bounds = cv::Scalar(20, -40);
  int adapt_hsv_patch_size_ = 25;
  cv::Mat threshim1, threshim2;
  std::vector<cv::Mat> channels;
  cv::split(input_image, channels);
  threshim1 = channels[1] < bounds[0];
  adaptiveThreshold(channels[2], threshim2, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, adapt_hsv_patch_size_, bounds[1]);
  output_image = (threshim1 & threshim2);
}

// @brief Return binary image of contours in "image",
//  @param image is an image Matrix
//  @param min_size is the the minimum size of the contours in the image
//  @return is the resulting binary image thresholded by min_size

cv::Mat LaneDetection::GetContours(const cv::Mat &image) {
  int blob_size_ = 100;
  //Draw onto a blank image based on found contours
  cv::Mat filtered(image.size(), CV_8UC1, cv::Scalar(0));
  std::vector<std::vector<cv::Point> > contours;
  cv::Mat copy(image.clone());
  findContours(copy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cv::Point());
  cv::Scalar color(255);
  for (size_t i = 0; i < contours.size(); i++) {
    if (contourArea(contours[i]) >= blob_size_) {
      drawContours(filtered, contours, i, color, -1, 8, cv::noArray(), 2, cv::Point());
    }
  }
  return filtered;
}

void LaneDetection::process_image(const cv::Mat &img_bgr8_, cv::Mat &out_, int left_or_right_){
  cv::Mat mask_warped_1_;
  cv::Mat mask_warped_2_;
  cv::Mat BEV_image_;
  if(left_or_right_ == 0)
    src = (cv::Mat_<float>(4,2) << 980.0, 520.0, 990.0, 480.0, 1280.0, 580.0, 1280.0, 660.0);
  else
    src = (cv::Mat_<float>(4,2) << 330.0, 0.0, 900.0, 0.0, 900.0, 710.0, 300.0, 710.0);
  get_BEV_image(img_bgr8_, BEV_image_, src);

  // Publish to a topic for debugging
  cv_bridge::CvImage img_bridge_output;
  std_msgs::Header header;
  header.stamp=ros::Time::now();
  img_bridge_output=cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, BEV_image_);
  
  if (left_or_right_ == 0)
  {
    test_left_warp_img.publish(img_bridge_output.toImageMsg());
  }
  else 
  {
    test_right_warp_img.publish(img_bridge_output.toImageMsg());
  }
  
  Multithreshold(BEV_image_, mask_warped_1_);
  FindWhite(BEV_image_, mask_warped_2_);
  cv::bitwise_or(mask_warped_1_, mask_warped_2_, mask_warped_1_);
  out_ = GetContours(mask_warped_1_);
}
