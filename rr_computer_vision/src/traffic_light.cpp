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
#include "thresholding_values.hpp"

// ROS includes
// #include <ros/console.h>
#include <std_srvs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

TrafficLightDetection::TrafficLightDetection(ros::NodeHandle nh) : it_(nh_), red_light_detected_(false), default_pixel_ratio_(0),
red_light_counter_(0), green_light_counter_(0), race_started(false) {
  img_subscriber_ = it_.subscribe(rr_sensor_topics::zed_right, 1, &TrafficLightDetection::ImgCallback, this);
  client_ = nh_.serviceClient<std_srvs::Empty>(rr_supervisor::start_race_service);
  // test_publisher_ = it_.advertise("/test_traffic_light", 1);
  // test_blob_publisher_ = it_.advertise("/test_blob_detection", 1);

  // Set up blob detector
  SetBlobDetectorParams();
  detector_ = cv::SimpleBlobDetector::create(params_);

  // Get hsv thresholding values based on the weather conditions
  int weather_condition;
  nh_.param<int>("WeatherCondition", weather_condition, 1);

  switch (weather_condition) {
    case 0:
      hsv_lower_bounds_ = traffic_light::overcast_hsv_lower_bounds;
      hsv_upper_bounds_ = traffic_light::overcast_hsv_upper_bounds;
      break;
    case 1:
      hsv_lower_bounds_ = traffic_light::sunny_hsv_lower_bounds;
      hsv_upper_bounds_ = traffic_light::sunny_hsv_upper_bounds;
      break;
    case 2:
      hsv_lower_bounds_ = traffic_light::sun_in_image_hsv_lower_bounds;
      hsv_upper_bounds_ = traffic_light::sun_in_image_hsv_upper_bounds;
      break;
    case 3:
      hsv_lower_bounds_ = traffic_light::indoor_hsv_lower_bounds;
      hsv_upper_bounds_ = traffic_light::indoor_hsv_upper_bounds;
      break;
  }
}

TrafficLightDetection::~TrafficLightDetection() {

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
  cv::Mat traffic_light_image;
  cv_bridge::CvImagePtr cv_ptr;
  std_srvs::Empty srv;
  double new_ratio = 0;

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    traffic_light_image = cv_ptr -> image;

    cv::Mat roi_img;
    CropImage(traffic_light_image, roi_img);

    cv::Mat threshold_img;
    RedColorThreshold(roi_img, threshold_img);

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
      }
      else {
        // green light buffer
        green_light_counter_++;
        if (green_light_counter_ >= green_frame_counter_max_) {
          ROS_INFO("Green light detected!");
          if (client_.call(srv)) {
            // Shutdown all ROS subscribers, publishers, and service clients
            img_subscriber_.shutdown();
            client_.shutdown();
            // test_publisher_.shutdown();
            // test_blob_publisher_.shutdown();

            // Set flag that cv node main can use
            race_started = true;
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
 * @brief Crop out the upper right quadrant of the image since the traffic light will always be in that area
 * @param input_img input image to be cropped
 * @param output_img cropped out image
 * @return void
 */
void TrafficLightDetection::CropImage(const cv::Mat& input_img, cv::Mat& output_img) {
  int width = input_img.size().width;
  int height = input_img.size().height;
  cv::Rect roi(640, 0, 640, 360);
  cv::Mat test = input_img(roi);
  output_img = test.clone();
}

/**
 * @brief Red color threshold in the HSV colour space
 * @param input_img input image in the bgr colour space
 * @param threshold_img output binary image
 * @return void
 */
void TrafficLightDetection::RedColorThreshold(const cv::Mat& input_img, cv::Mat& output_img) {
  cv::Mat hsv_img;
  cv::cvtColor(input_img, hsv_img, CV_BGR2HSV);
  cv::inRange(hsv_img, hsv_lower_bounds_, hsv_upper_bounds_, output_img);
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

  // Draw blobs on image
  // cv::Mat test(threshold_img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
  // cv::cvtColor(threshold_img, test, CV_GRAY2BGR);
  // cv::Mat im_with_keypoints;
  // cv::drawKeypoints( test, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

  // Publish to rviz (for debugging)
  // cv_bridge::CvImage img_bridge_output;
  // std_msgs::Header header;
  // header.stamp = ros::Time::now();
  // img_bridge_output = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, im_with_keypoints);
  // test_blob_publisher_.publish(img_bridge_output.toImageMsg());

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
    // ROS_INFO("NUM BLOBS: %d", num_blob);
    // ROS_INFO("MAX BLOB AREA: %d", total_pixel);
    
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
    
    if (red_light_counter_ >= red_frame_counter_max_) {
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