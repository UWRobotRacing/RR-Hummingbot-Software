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
    zed_subscriber = nh_.subscribe("/zed/rgb/image_rect_color", 1, &LaneDetection::RGBCameraCallback, this);
    //test_subscriber = nh_.subscribe("/test_publisher", 1, &LaneDetection::TestCallback, this);
}

void LaneDetection::InitializePublishers() {
    test_publisher = nh_.advertise<sensor_msgs::Image>("/test_publisher", 1);
}

void LaneDetection::RGBCameraCallback(const sensor_msgs::Image& msg){

/*
    cv::bitwise_or(mask_warped_1_, mask_warped_2_, mask_warped_1_);

    if (!((mask_.cols == mask_warped_1_.cols) && (mask_.rows == mask_warped_1_.rows)))
    { 
      mask_ = cv::Mat(im_input_.rows, im_input_.cols, CV_8U, cv::Scalar::all(255));
      cv::warpPerspective(mask_, mask_, transform_matrix_, BEV_size_);
    }

    out = GetContours(mask_warped_1_, blob_size_);
*/
    // Convert sensor_msgs::Image to a BGR8 cv::Mat
    cv_bridge::CvImagePtr cv_bridge_bgr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img_bgr8 = cv_bridge_bgr->image;
    cv::cvtColor(img_bgr8, Im1_HSV_, CV_BGR2HSV, 3);
    //cv::cvtColor(img_bgr8, img_gray, cv::COLOR_BGR2GRAY);

    src = (cv::Mat_<float>(4,2) << 500.0, 400.0, 750.0, 400.0, 1280.0, 600.0, 0, 600.0);
    dst = (cv::Mat_<float>(4,2) << 300.0, 0, 900.0, 0, 900.0, 730.0, 300.0, 730.0);

    ROS_INFO("BUMP");
    
    M = cv::getPerspectiveTransform(src, dst);
    cv::warpPerspective(img_gray,BEV_image,M,img_gray.size(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    multibounds_ = (cv::Mat_<double>(3,6) << 0, 100, 140, 120, 255, 255, 0, 0, 250, 255, 25, 255, 25, 5, 186, 130, 50, 255);
    Multithreshold(BEV_image, multibounds_, mask_warped_1_);

    bounds_ = cv::Scalar(20, -40);
    adapt_hsv_patch_size_ = 25;
    FindWhite(BEV_image, bounds_, adapt_hsv_patch_size_, mask_warped_2_);

    cv_bridge::CvImage img_bridge_output;
    std_msgs::Header header; // empty header
    header.stamp = ros::Time::now(); // time
    img_bridge_output = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, Im1_HSV_);
    
    // Publish 
    test_publisher.publish(img_bridge_output.toImageMsg());
}


void Multithreshold(const cv::Mat &input_image, const cv::Mat &bounds, cv::Mat &output_image) {
  //Confirm Dims and define binary mask_
  cv::Mat mask_(input_image.rows, input_image.cols, CV_8U, cv::Scalar::all(0));
  mask_.copyTo(output_image);

  // define min/max for proper wrapping
  double max = DBL_MAX;
  double min = -DBL_MAX;

  // loop through threshold bands, seperate into lower & upper thresholds, if any channel has lower > upper,
  // split into 2 bands, else keep original. 
  for (int i = 0; i < bounds.rows; i++)
  {
    cv::Mat lowerbound(bounds.colRange(0, bounds.cols / 2).rowRange(i, i + 1));
    cv::Mat upperbound(bounds.colRange(bounds.cols / 2, bounds.cols).rowRange(i, i + 1));
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

void FindWhite(const cv::Mat &input_image, const cv::Scalar bounds, int patch_size, cv::Mat &output_image) {
  cv::Mat threshim1, threshim2;
  std::vector<cv::Mat> channels;
  cv::split(input_image, channels);
  threshim1 = channels[1] < bounds[0];
  adaptiveThreshold(channels[2], threshim2, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, patch_size, bounds[1]);
  output_image = (threshim1 & threshim2);
}

// @brief Return binary image of contours in "image",
//  @param image is an image Matrix
//  @param min_size is the the minimum size of the contours in the image
//  @return is the resulting binary image thresholded by min_size

cv::Mat GetContours(const cv::Mat &image, int min_size) {
  //Draw onto a blank image based on found contours
  cv::Mat filtered(image.size(), CV_8UC1, cv::Scalar(0));
  std::vector<std::vector<cv::Point> > contours;
  cv::Mat copy(image.clone());
  findContours(copy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cv::Point());
  cv::Scalar color(255);
  for (size_t i = 0; i < contours.size(); i++) {
    if (contourArea(contours[i]) >= min_size) {
      drawContours(filtered, contours, i, color, -1, 8, cv::noArray(), 2, cv::Point());
    }
  }
  return filtered;
}
