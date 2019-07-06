/**
 * @file traffic_light.cpp
 * @brief Traffic Light cpp File
 * @author Yuchi(Allan) Zhao
 * @author Waleed Ahmed (w29ahmed)
 * @competition IARRC 2019
 */

// Local includes
#include "traffic_light.hpp"
#include "rr_topic_names.hpp"

// ROS includes
// #include <ros/console.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

TrafficLightDetection::TrafficLightDetection(ros::NodeHandle nh) : it_(nh_), red_light_detected_(false), default_pixel_ratio_(0),
red_light_counter_(0), green_light_counter_(0) {
  img_subscriber_ = it_.subscribe(rr_sensor_topics::zed_right, 1, &TrafficLightDetection::ImgCallback, this);
  client_ = nh_.serviceClient<std_srvs::Empty>("/Supervisor/start_race");
  // test_publisher_ = it_.advertise("/test_traffic_light", 1);

  // Set up blob detector
  SetBlobDetectorParams();
  detector_ = cv::SimpleBlobDetector::create(params_);
}

/**
 * @brief Sets parameters for OpenCV's simple blob detector class such that it will catch only circlar blobs
 * @return void
 */
void TrafficLightDetection::SetBlobDetectorParams() {
  // Filter by Area
  params_.filterByArea = true;
  params_.minArea = 250;
        
  // Filter by Circularity
  params_.filterByCircularity = true;
  params_.minCircularity = 0.6;
        
  // Filter by Convexity
  params_.filterByConvexity = true;
  params_.minConvexity = 0.6;
        
  // Filter by Inertia
  params_.filterByInertia = false;
  //params.minInertiaRatio = 0.01;

  // Filter white
  params_.blobColor = 255;
}

void TrafficLightDetection::ImgCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat traffic_light_image, hsv_img, threshold_img, blur_img, im_with_keypoints;
  cv_bridge::CvImagePtr cv_ptr;
  std_srvs::Trigger srv;
  double new_ratio = 0;

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    traffic_light_image = cv_ptr -> image;

    RedColorThreshold(traffic_light_image, threshold_img);
    // PUBLISH and visualize in rviz
    // cv_bridge::CvImage img_bridge_output;
    // std_msgs::Header header;
    // header.stamp = ros::Time::now();
    // img_bridge_output = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, threshold_img);
    // test_publisher_.publish(img_bridge_output.toImageMsg());

    if (red_light_detected_ == false) {
      RedLightDetection(threshold_img);
    }
    else {
      // crop the existing rectangle on each new frame and count pixel
      cv::Mat crop_img = threshold_img(boundRect_);
      int red_Pixel_Counter = cv::countNonZero(crop_img);
      int total_pixel = crop_img.total();
      double pixel_ratio = (double) red_Pixel_Counter / total_pixel;
      // ROS_INFO("pixel_ratio: %f", pixel_ratio);

      if ((pixel_ratio + pixel_ratio_range_) >= default_pixel_ratio_) {
        green_light_counter_ = 0;
        ROS_INFO("Red light is still there.");
      }
      else {
        // green light buffer
        green_light_counter_++;
        if (green_light_counter_ >= frame_counter_max_) {
          ROS_INFO("Green light detected!");
          if (client_.call(srv)) {
            ros::shutdown();
          }
        }
      }
    }
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

/**
 * @brief Red color threshold in the HSV colour space
 * @param input_img input image in the bgr colour space
 * @param threshold_img output binary image
 * @return void
 */
void TrafficLightDetection::RedColorThreshold(const cv::Mat& input_img, cv::Mat& threshold_img) {
  cv::Mat hsv_img;
  cv::cvtColor(input_img, hsv_img, CV_BGR2HSV);
  cv::inRange(hsv_img, cv::Scalar(0, 90, 155), cv::Scalar(15, 255, 255), threshold_img);
  cv::GaussianBlur(threshold_img, threshold_img, cv::Size(7,7), 0, 0);
}

/**
 * @brief Function that oversees the red light detection algorithm
 * @param threshold_img Binary image after a red threshold
 * @return void
 */
void TrafficLightDetection::RedLightDetection(const cv::Mat& threshold_img) {
  // Detect blobs.
  std::vector<cv::KeyPoint> keypoints;
  detector_ -> detect( threshold_img, keypoints);
  int num_blob = keypoints.size();
  cv::Mat crop_img;
  int max_area_index = 0;

  if (num_blob > 0) {
    // find the largest blob if there are more than 2 blobs
    if (num_blob > 1) {
      for (int i = 0; i < num_blob; i++) {
        if (keypoints[max_area_index].size < keypoints[i+1].size) {
          max_area_index = i+1;
        }
      }
    }

    FindBoundRect(threshold_img, crop_img, keypoints, max_area_index);
    // count pixel in the rectangle
    int red_Pixel_Counter = cv::countNonZero(crop_img);
    int total_pixel = crop_img.total();
    // ROS_INFO("BLOB AREA: %d", total_pixel);
    
    if (red_light_counter_ == 0) {
      default_pixel_ratio_ = (double)red_Pixel_Counter/total_pixel;
      // ROS_INFO("Reference Ratio: %f", default_pixel_ratio_);
      red_light_counter_++;
    }
    else {
      double pixel_ratio = (double)red_Pixel_Counter/total_pixel;

      // red light buffer             
      if ((pixel_ratio + pixel_ratio_range_) >= default_pixel_ratio_) {
        red_light_counter_++;
      }
    }
    
    if (red_light_counter_ >= frame_counter_max_) {
      ROS_INFO("Red light detected!");
      red_light_detected_ = true;
      red_light_counter_ = 0;
    }
  }
}

/**
 * @brief Find a bounding box around the largest blob in a binary image containing blobs
 * @param threshold_img input binary image containing blobs
 * @param crop_img output binary image containing the largest blob cropped out
 * @param keypoints vector of points that describe all the blobs in the image (output from blob detection)
 * @param max_area_index index in the keypoints vector of the largest blob
 * @return void
 */
void TrafficLightDetection::FindBoundRect(const cv::Mat& threshold_img, cv::Mat& crop_img, std::vector<cv::KeyPoint>& keypoints, const int max_area_index) {
  // the center and diameter of the blob
  int blob_x = (int)keypoints[max_area_index].pt.x;
  int blob_y = (int)keypoints[max_area_index].pt.y;
  int blob_diameter = (int)keypoints[max_area_index].size;
  int blob_radius = (int)blob_diameter/2;

  // create a reatangle around the blob
  cv::Rect firstRect (blob_x-blob_radius, blob_y-blob_radius, blob_diameter, blob_diameter);
  crop_img = threshold_img(firstRect);
  boundRect_ = firstRect;
}