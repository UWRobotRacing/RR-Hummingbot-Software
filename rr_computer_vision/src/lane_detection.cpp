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
    test_subscriber = nh_.subscribe("/test_publisher", 1, &LaneDetection::TestCallback, this);
}

void LaneDetection::InitializePublishers() {
    test_publisher = nh_.advertise<sensor_msgs::Image>("/test_publisher", 1);
}

void LaneDetection::RGBCameraCallback(const sensor_msgs::Image& msg){
/*
    cvtColor(im_input_, Im1_HSV_, CV_BGR2HSV, 3);
    cv::warpPerspective(Im1_HSV_, Im1_HSV_warped_, transform_matrix_, BEV_size_, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    Multithreshold(Im1_HSV_warped_, multibounds_, mask_warped_1_);
    FindWhite(Im1_HSV_warped_, bounds_, adapt_hsv_patch_size_, mask_warped_2_);
    cv::bitwise_or(mask_warped_1_, mask_warped_2_, mask_warped_1_);

    if (!((mask_.cols == mask_warped_1_.cols) && (mask_.rows == mask_warped_1_.rows)))
    { 
      mask_ = cv::Mat(im_input_.rows, im_input_.cols, CV_8U, cv::Scalar::all(255));
      cv::warpPerspective(mask_, mask_, transform_matrix_, BEV_size_);
    }

    out = GetContours(mask_warped_1_, blob_size_);

    out.copyTo(cv_output_bridge_.image);
    cv_output_bridge_.encoding = "mono8";
    test_publisher.publish(cv_output_bridge_.toImageMsg());
*/
    cv::Mat camera_data = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat mask_(camera_data.rows, camera_data.cols, CV_8U, cv::Scalar::all(255));
    mask_.copyTo(camera_data);

    //cv::Canny(camera_data, camera_data, 30, 70, 3);
    // cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    /*
    ros::Time time = ros::Time::now();
    cv_bridge::CvImagePtr cvImage(new cv_bridge::CvImage);
    cvImage->encoding = "bgr8";
    cvImage->header.stamp = time;
    cvImage->header.frame_id = "/test_publisher";
    cvImage->image = camera_data;
    test_publisher.publish(cvImage->toImageMsg());
*/
    cv::namedWindow( "Display window", 50);// Create a window for display.
    cv::imshow( "Display window", camera_data); 
}

void LaneDetection::TestCallback(const sensor_msgs::Image& msg) {
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
