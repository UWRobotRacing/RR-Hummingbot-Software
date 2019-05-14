/** @file lane_detection.cpp
 *  @author Andrew Jin
 *  @competition IARRC 2019
 */

#include "lane_detection.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>


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
    test_subscriber = nh_.subscribe("/zed/rgb/image_rect_color", 0, &LaneDetection::RGBCameraCallback, this);
}

void LaneDetection::InitializePublishers() {
    test_publisher = nh_.advertise<sensor_msgs::Image>("/test_publisher", 10);
}

void LaneDetection::RGBCameraCallback(const sensor_msgs::Image& msg){
    cv_bridge::CvImagePtr cv_input_bridge_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat im_input_;
    cv_input_bridge_->image.copyTo(im_input_);
    cv::Mat detected_edges;
    sensor_msgs::Image output;
    cv::Size Size;
    //cv::Canny(im_input_, detected_edges, 50, 80, 3);
    cv::blur(im_input_, detected_edges, cv::Size(3,3));

    cv_bridge::CvImage out_msg;
    out_msg.header   = msg.header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
    out_msg.image    = detected_edges; // Your cv::Mat

    test_publisher.publish(out_msg.toImageMsg());
    ros::spin();
}

/*
void lane_detection_processor::FindLanes(const sensor_msgs::Image::ConstPtr &msg)
{
  try
  {
    cv_input_bridge_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv_input_bridge_->image.copyTo(im_input_);

    cvtColor(im_input_, Im1_HSV_, CV_BGR2HSV, 3);
    cv::warpPerspective(Im1_HSV_, Im1_HSV_warped_, transform_matrix_, BEV_size_, cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    Multithreshold(Im1_HSV_warped_, multibounds_, mask_warped_1_);
    FindWhite(Im1_HSV_warped_, bounds_, adapt_hsv_patch_size_, mask_warped_2_);
    cv::bitwise_or(mask_warped_1_, mask_warped_2_, mask_warped_1_);

    // sets up the BEV mask for that camera to remove everything outside of them
    // masked area
    if (!((mask_.cols == mask_warped_1_.cols) && (mask_.rows == mask_warped_1_.rows)))
    {
      mask_ = cv::Mat(im_input_.rows, im_input_.cols, CV_8U, cv::Scalar::all(255));
      cv::warpPerspective(mask_, mask_, transform_matrix_, BEV_size_);
    }

    cv::Mat out;               // dst must be a different Mat

  
    out = GetContours(mask_warped_1_ &mask_, blob_size_);
  

    //find mask_
    //Copy to output bridge
    out.copyTo(cv_output_bridge_.image);
    cv_output_bridge_.encoding = "mono8";

    //Input Image has been processed and published
    image_pub_.publish(cv_output_bridge_.toImageMsg());

    if (point_out_)
    {
      occupancy_.clear();
      occupancy_.reserve(out.cols * out.rows);

      data_pointer_ = out.data;
      for (int i = 0; i < out.rows * out.cols; i++)
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
      grid_msg_.info = meta_data_;
      pointList_pub_.publish(grid_msg_);
    }
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
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

*/