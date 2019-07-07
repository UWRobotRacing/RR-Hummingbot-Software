/** @file lane_detection.cpp
 *  @author Waleed Ahmed (w29ahmed)
 *  @author Carson Chen
 *  @author Matthew Post
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2019
 */

// Standard includes
#include <vector>

// Local includes
#include "rr_topic_names.hpp"
#include "lane_detection.hpp"

// ROS includes
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

LaneDetection::LaneDetection(ros::NodeHandle nh) : it_(nh_) {
  // Extract parameters from yaml file
  std::string params_file;
  nh.param<std::string>("LaneDetectionParamsFile", params_file, "drag_race.yaml");
  cv::FileStorage fs(params_file, cv::FileStorage::READ);
  fs["warp_src_coords"] >> warp_src_coords_;
  fs["camera_feed"] >> camera_feed_;
  fs["min_contour_size"] >> min_contour_size_;

  fs["resolution"] >> resolution_;
  fs["camera_width_offset"] >> camera_width_offset_; 
  fs["camera_height_offset"] >> camera_height_offset_;

  // Occupancy grid meta data
  meta_data_.height = 720;  
  meta_data_.width = 1280;
  meta_data_.resolution = resolution_; 
  meta_data_.origin.position.x = 0;
  meta_data_.origin.position.y = 0;

  // Initialize publishers and subscribers
  InitializeSubscribers();  

  InitializePublishers();
}

LaneDetection::~LaneDetection() {

}

void LaneDetection::InitializeSubscribers() {
  std::string camera_topic = (!camera_feed_.compare("right")) ? rr_sensor_topics::zed_right : rr_sensor_topics::zed_left;
  img_subscriber_ = it_.subscribe(camera_topic, 1, &LaneDetection::ImgCallback, this);
}

void LaneDetection::InitializePublishers() {
  // Setup debug rostopics
  // test_thres_img_pub_ = nh_.advertise<sensor_msgs::Image>("/test_thres_img", 1);
  // test_warp_img_pub_ = nh_.advertise<sensor_msgs::Image>("/test_warp_img", 1);
  // test_contour_filter_img_pub_ = nh_.advertise<sensor_msgs::Image>("/test_contour_filter_img", 1);

  grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(rr_cv::lane_detection_occupancy_grid, 1);
}

void LaneDetection::ImgCallback(const sensor_msgs::ImageConstPtr& msg) {
  // Convert sensor_msgs::Image to a BGR8 cv::Mat
  cv::Mat img_input_bgr8;
  cv_bridge::CvImagePtr cv_bridge_bgr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  img_input_bgr8 = cv_bridge_bgr->image;  
  
  cv::Mat img_thres;
  Threshold(img_input_bgr8, img_thres);
  // Publish thresholded image (For testing purposes)
  // cv_bridge::CvImage img_bridge_output;

  // std_msgs::Header header;
  // header.stamp=ros::Time::now();
  // img_bridge_output = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img_thres);
  // test_thres_img_pub_.publish(img_bridge_output.toImageMsg());

  // cv::cuda::GpuMat img_warp_input(img_thres);
  // cv::cuda::GpuMat img_warp_output;

  cv::Mat img_warp;
  Warp(img_thres, img_warp, warp_src_coords_);

  // Publish warped image (For testing purposes)
  // header.stamp=ros::Time::now();
  // img_bridge_output = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img_warp);
  // test_warp_img_pub_.publish(img_bridge_output.toImageMsg());

  cv::Mat img_contour_output;
  // cv::Mat img_contour_input = img_warp_output;
  img_contour_output = ContourFilter(img_warp, min_contour_size_);

  // Publish contour filtered image (For testing purposes)
  // header.stamp=ros::Time::now();
  // img_bridge_output = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img_contour_output);
  // test_contour_filter_img_pub_.publish(img_bridge_output.toImageMsg());

  nav_msgs::OccupancyGrid grid_msg;
  ConvertToOccupancyGrid(img_contour_output, grid_msg);
  grid_pub_.publish(grid_msg);
}  

/**
 * @brief Perform an adaptive threshold on the value channel in the hsv colour space
 * @param input_img input image in the bgr colour space
 * @param output_img output binary image
 * @return void
 */
void LaneDetection::Threshold(const cv::Mat &input_img, cv::Mat &output_img) {
  cv::cvtColor(input_img, input_img, CV_BGR2HSV, 3);
  int patch_size_ = 25; 
  std::vector<cv::Mat> channels;
  cv::split(input_img, channels);
  cv::adaptiveThreshold(channels[2], output_img, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY,patch_size_, -20);
}

/**
 * @brief Applies a perspective warp transform
 * @param input_img input image
 * @param output_img output image with warp transform applied
 * @param src warp transform source image co-ordinates
 * @return void
 */
void LaneDetection::Warp(const cv::Mat &input_img, cv::Mat &output_img, const cv::Mat src) {
  cv::Mat dst = (cv::Mat_<float>(4,2) << 300.0, 0, 900.0, 0, 900.0, 710.0, 300.0, 710.0);
  cv::Mat transform_matrix = cv::getPerspectiveTransform(src, dst);
  cv::warpPerspective(input_img, output_img, transform_matrix, input_img.size());
}

/**
 * @brief Filters out contours from an image that are below a specified minimum contour size
 * @param img input image to filter contours from
 * @param min_contour_size contour size that each contour found has to be above to stay in the image
 * @return image with only contours in it that are above the minimum contour size
 */
cv::Mat LaneDetection::ContourFilter(const cv::Mat &img, const int min_contour_size) {
  // Make a blank image with the same dimensions as the input image
  cv::Mat filtered(img.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat copy(img.clone());

  // Find contours in the image
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(copy, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point());

  // Only draw contours on the blank image that are above the minimum contour size
  cv::Scalar color(255);
  for (size_t i = 0; i < contours.size(); i++) {
    if (cv::contourArea(contours[i]) >= min_contour_size) {
      cv::drawContours(filtered, contours, i, color, CV_FILLED , 8, cv::noArray(), 2, cv::Point());
    } 

  }
  return filtered;
}

/**
 * @brief Convert a binary opencv image to a ros occupancy grid
 * @param img input binary opencv matrix
 * @param grid_msg ros occupancy grid message object to dump the data into
 * @return void
 */
void LaneDetection::ConvertToOccupancyGrid(const cv::Mat &img, nav_msgs::OccupancyGrid &grid_msg) {
  uchar* data_pointer_; 
  std::vector<int8_t> occupancy_;
  occupancy_.clear();
  occupancy_.reserve(img.cols * img.rows); 

  for (int row = img.rows; row > 0; row--) 
  {
    for (int col = 0; col < img.cols; col++) 
    {
      if (img.at<uchar>(row, col) == 0) 
      {
        // Unknown cell
        occupancy_.push_back(-1);
      }
      else
      {
        // Occupied cell
        occupancy_.push_back(100); 
      }
    }
  }

  grid_msg.data = occupancy_; 

  /*
  meta_data_.origin.position.x = -1*(meta_data_.width*meta_data_.resolution)/2 - camera_width_offset_;
  meta_data_.origin.position.y = -camera_height_offset_;
   */

  grid_msg.info = meta_data_; 
}