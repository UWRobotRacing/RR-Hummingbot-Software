/* 
 *  @file sign_detection.cpp
 *  @brief Sign Detection Implementation Class
 *
 * Topics Subscribed:
 *   rr_sensor_topics/red_right
 *   rr_cv/horizontal_lane_monitor
 *
 * Topics Published:
 *   rr_cv/traffic_sign_status
 *
 *  @author Martin Ethier (MartinEthier)
 *  @competition IARRC 2019
 */

//LOCAL
#include "sign_detection.hpp"
#include "rr_topic_names.hpp"
#include <vector>
#include <math.h>

// OpenCV includes
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// Ros includes
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/**
 * @brief Finds bounding box with the highest level weight value
 * @param best_bbox found bbox with highest level weight
 * @param bounding_boxes vector of rects to check
 * @param level_weights vector of weights for each bbox
 * @return void
 */
void get_best_bounding_box(cv::Rect& best_bbox, std::vector<cv::Rect>& bounding_boxes, std::vector<double> level_weights) {
  std::vector<int>::size_type best_index = 0;
  double best_weight = -100.0;

  // Loop through bboxes, updating best weight each iteration
  for(std::vector<int>::size_type i = 0; i < bounding_boxes.size(); i++) {
    if (level_weights[i] > best_weight) {
      best_weight = level_weights[i];
      best_index = i;
    }
  }
  best_bbox = bounding_boxes[best_index];
}

/**
 * @brief Gets index of contour nearest to the center
 * @param sign image in which contours belong to
 * @param contours vector of contours within the image
 * @return int index of contour with smallest distance to center
 */
int get_centered_contour(const cv::Mat& img, std::vector<std::vector<cv::Point>> contours) {
  // Find image center and initialize smallest_dist to twice the size of the input image
  cv::Point2f image_center(img.size().width / 2, img.size().height / 2);
  double smallest_dist = std::max(img.size().width, img.size().height) * 2;
  int sd_index = 0;

  for (int i = 0; i < contours.size(); i++) {
    // Get centroid of contour
    cv::Moments mom = cv::moments(contours[i], true);

    // Compute center of moment, add 1e-5 to avoid division by zero
    cv::Point2f moment_center(mom.m10 / (mom.m00 + 1e-5), mom.m01 / (mom.m00 + 1e-5));

    // Get euclidean distance from image center to moment center
    double dist = cv::norm(moment_center - image_center);

    if (dist < smallest_dist) {
      smallest_dist = dist;
      sd_index = i;
    }
  }

  return sd_index;
}

/**
 * @brief Expand a cv::Rect by a constant ratio
 * @param rect rectangle to be expanded
 * @param img_size size of image the rect is cut from
 * @return void
 */
void expand_rect(cv::Rect& rect, cv::Size img_size) {
  // Set ratio at which to expand bbox edges by
  float percent_expand = 1.2;

  // Increase rect size and recenter using an offset
  cv::Size delta_size(rect.width * percent_expand, rect.height * percent_expand);
  cv::Point offset(delta_size.width / 2, delta_size.height / 2);
  rect += delta_size;
  rect -= offset;

  // Check if rect crosses any image boundaries
  if (rect.x < 0) {
    rect.x = 0;
  }
  if (rect.x + rect.width > img_size.width) {
    rect.width = img_size.width - rect.x;
  } 
  if (rect.y < 0) {
    rect.y = 0;
  } 
  if (rect.y + rect.height > img_size.height) {
    rect.height = img_size.height - rect.y;
  }
}

/*
 * @name SignDetection 
 * @brief Constructor
 * @param nh: ROS node handler
 * @return NONE
*/
SignDetection::SignDetection(ros::NodeHandle nh) : it_(nh_) {
  InitializeSubscribers();  
  InitializePublishers();
  
  // Get classifier location
  std::string cascade_file;
  nh.param<std::string>("HaarCascadeFile", cascade_file, "");

  // Load in classifier
  sign_cascade.load(cascade_file);
  if( sign_cascade.empty() )
    {
        ROS_ERROR("ERROR LOADING CLASSIFIER");
    };
}

void SignDetection::InitializeSubscribers() {
  zed_right_img_sub_ = it_.subscribe(rr_sensor_topics::zed_right, 1, &SignDetection::ZedCameraImgCallback, this);
  horiz_lane_monitor_sub_ = nh_.subscribe(rr_cv::horizontal_lane_monitor, 1, &SignDetection::HorizontalLaneMonitorCallback, this);
}

void SignDetection::InitializePublishers() {
  // Setup debug rostopics
  // img_publisher_ = it_.advertise("/test_cascade", 1);

  sign_status_pub_ = nh_.advertise<rr_computer_vision::TrafficSign>(rr_cv::traffic_sign_status, 1);
}

void SignDetection::HorizontalLaneMonitorCallback(const std_msgs::Bool& lane_crossed_msg) {
  // Convert Bool msg to a bool and assign to variable
  horizontal_lane_crossed = lane_crossed_msg.data;
}

void SignDetection::ZedCameraImgCallback(const sensor_msgs::ImageConstPtr& right_img_msg) {
  // Convert sensor_msgs::Image to a BGR8 cv::Mat 
  cv::Mat rgb_img = (cv_bridge::toCvCopy(right_img_msg, sensor_msgs::image_encodings::BGR8))->image;

  // Cut out bottom half and left quarter of image to speed up node
  cv::Rect crop(rgb_img.cols / 4, 0, rgb_img.cols * 3/4 - 1, rgb_img.rows / 2);
  rgb_img = rgb_img(crop);
  
  // Convert to grayscale and equalize
  cv::Mat right_img;
  cv::cvtColor(rgb_img, right_img, cv::COLOR_BGR2GRAY);
  cv::equalizeHist(right_img, right_img);

  // Apply classifier to grayscale image
  std::vector<cv::Rect> classifications;
  std::vector<int> reject_levels;
  std::vector<double> level_weights;
  sign_cascade.detectMultiScale(right_img, classifications, reject_levels, level_weights, 1.05, 3, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(20, 20), cv::Size(), true);
  
  cv::Rect best_bbox;
  if (classifications.empty()) {
    consecutive_frames = 0;
  } else {
    // Find the best bbox from cascade output
    get_best_bounding_box(best_bbox, classifications, level_weights);
    
    // Check if bounding box from last frame overlaps with current frame
    if (consecutive_frames != 0) {
      bool overlaps = ((best_bbox & last_frame_bbox).area() > 0);
      if (overlaps) {
        consecutive_frames += 1;
        last_frame_bbox = best_bbox;
      } else {
        consecutive_frames = 0;
      }
    } else {
      consecutive_frames += 1;
      last_frame_bbox = best_bbox;
    }
    // Add bbox to rgb_img for debugging
    // cv::rectangle(rgb_img, best_bbox, cv::Scalar(0, 0, 255));
  }
  
  rr_computer_vision::TrafficSign sign_status_msg;
  if (check_arrow) {
    if (horizontal_lane_crossed) {
      // Get arrow direction by finding the index of the max value in the accumulator
      int direction = std::max_element(arrow_status_accumulator.begin(), arrow_status_accumulator.end()) - arrow_status_accumulator.begin();

      // Publish the found direction
      sign_status_msg.traffic_sign_status = direction;
      sign_status_pub_.publish(sign_status_msg);
      check_arrow = false;
      
      // Reset status accumulator to all zeros
      std::fill(arrow_status_accumulator.begin(), arrow_status_accumulator.end(), 0);
    } else {
      if (!best_bbox.empty()) {
        // Expand rectangle to ensure we get entire sign
        expand_rect(best_bbox, right_img.size());

        // Check arrow direction and add to accumulator
        uint8_t arrow_status = CheckArrowDir(cv::Mat(right_img, best_bbox).clone());
        ++arrow_status_accumulator[arrow_status];
      }
      sign_status_msg.traffic_sign_status = NONE;
      sign_status_pub_.publish(sign_status_msg);
    }

  } else {
    if (consecutive_frames > 10) {
      // Only start checking arrow direction after 10 consecutive sign frames
      check_arrow = true;
    }
    sign_status_msg.traffic_sign_status = NONE;
    sign_status_pub_.publish(sign_status_msg);
  }

  // Publish so we can visualize in rviz
  // cv_bridge::CvImage img_bridge_output;
  // std_msgs::Header header; // empty header
  // header.stamp = ros::Time::now(); // time
  // img_bridge_output = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, rgb_img);
  // img_publisher_.publish(img_bridge_output.toImageMsg());
}

/**
 * @brief Determine the direction of the arrow located in the given sign
 * @param sign cropped out image of the sign
 * @return uint8_t sign direction according to enum defined in header
 */
uint8_t SignDetection::CheckArrowDir(const cv::Mat& sign) {
  // Compute parameters for canny by using Otsu thresholding output
  cv::Mat thresh_out;
  double high_thresh = cv::threshold(sign, thresh_out, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  double low_thresh = 0.3 * high_thresh;
  const int kernel_size = 3;

  // Apply canny edge detection
  cv::Mat canny_out;
  cv::GaussianBlur(sign, canny_out, cv::Size(3,3), 0, 0);
  cv::Canny(canny_out, canny_out, low_thresh, high_thresh, kernel_size);

  // Apply a morphological closing operation to close open edges
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
  cv::morphologyEx(canny_out, canny_out, cv::MORPH_CLOSE, kernel);

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> arrow_hierarchy;
  cv::findContours(canny_out, contours, arrow_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  
  // Get index of contour nearest to image center
  int sd_index = get_centered_contour(sign, contours);

  // Create arrow matrix
  cv::Mat arrow = cv::Mat::zeros(sign.size(), CV_8UC1);
  cv::drawContours(arrow, contours, sd_index, cv::Scalar(255), CV_FILLED);

  // Create bgr arrow for debugging images
  // cv::Mat arrow_bgr;
  // cv::cvtColor(arrow, arrow_bgr, cv::COLOR_GRAY2BGR);

  // Fit an ellipse to the arrow and get some parameters from it
  cv::RotatedRect rect_fit = cv::fitEllipse(contours[sd_index]);
  float angle = rect_fit.angle;
  cv::Size2f rect_size = rect_fit.size;
  cv::Point2f rect_center = rect_fit.center;

  // If angle is vertical, conclude that arrow direction is straight
  // Angle ranges 0-180 deg from the vertival axis going CW
  if (angle < 45 || angle > 135) {
    // Sign is straight
    return STRAIGHT;
  } else {
    // Sign is horizontal, need to apply secondary check, summary of check is:
    // Get a line perpendicular to the orientation of the arrow that passes through it's middle
    // Use this line to create two masks that split the arrow down the middle
    // Mask the arrow and count the number of pixels on each side
    // The side with the most pixels will be the side with the arrow head
    
    // Get slope of minor axis
    float slope = tan((angle) * M_PI / 180);

    // Define line lambda function that passes through rect_center
    auto get_x = [&](int y) { return 1 / slope * (y - rect_center.y) + rect_center.x; };

    // Find corrdinates for points that go through top and bottom edges
    cv::Point top(get_x(0), 0);
    cv::Point bottom(get_x(arrow.rows), arrow.rows);

    // Create masks for left and right half of the arrow
    std::vector<cv::Point> left_mask_points = {bottom, cv::Point(0, arrow.rows-1), cv::Point(0, 0), top};
    std::vector<cv::Point> right_mask_points = {bottom, cv::Point(arrow.cols-1, arrow.rows-1), cv::Point(arrow.cols-1, 0), top};

    cv::Mat left_mask = cv::Mat::zeros(arrow.size(), CV_8U);
    cv::fillConvexPoly(left_mask, left_mask_points, cv::Scalar(255));

    // Draw masks on image with alpha blending for debugging
    // double alpha = 0.5;
    // cv::Mat overlay;
    // arrow_bgr.copyTo(overlay);
    // cv::fillConvexPoly(overlay, left_mask_points, cv::Scalar(255, 0, 0));
    // cv::fillConvexPoly(overlay, right_mask_points, cv::Scalar(0, 255, 0));
    // cv::addWeighted(overlay, alpha, arrow_bgr, 1 - alpha, 0, arrow_bgr);

    // Apply mask to arrow halves
    cv::Mat right_mask = cv::Mat::zeros(arrow.size(), CV_8U);
    cv::Mat left_arrow, right_arrow;
    arrow.copyTo(left_arrow, left_mask);
    cv::bitwise_not(left_mask, right_mask);
    arrow.copyTo(right_arrow, right_mask);

    // Count pixels in left and right arrows
    int left_area = cv::countNonZero(left_arrow);
    int right_area = cv::countNonZero(right_arrow);

    // Return arrow direction with highest area
    uint8_t status_out;
    if (left_area > right_area) {
      status_out = LEFT;
    } else {
      status_out = RIGHT;
    }

    return status_out;
  }
}