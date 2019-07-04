// Notes:
// cv::Rect bbox;
// bbox.x, bbox.y, bbox.width, bbox.height (x, y is top-left corner)

// Sign message:

// cut out bottom 1/4 of image before feeding into haar




/* 
 *  @file sign_detection.cpp
 *  @brief Sign Detection Implementation Class
 *
 * Topics Subscribed:
 *   /zed/left/image_rect_color
 *   /zed/depth/depth_registered
 *
 * Service called:
 *   /
 *
 *  @author Martin Ethier (MartinEthier)
 *  @competition IARRC 2019
 */

//LOCAL
#include "sign_detection.hpp"
#include <vector>
#include <math.h>

// OpenCV includes
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// Ros includes
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/*
 * @name SignDetection 
 * @brief Constructor
 * @param nh: ROS node handler
 * @return NONE
*/
SignDetection::SignDetection(ros::NodeHandle nh) : it_(nh_) {
  img_publisher_ = it_.advertise("/test_cascade", 1);

  img_subscriber_ = it_.subscribe("zed/zed_node/right/image_rect_color", 1, &SignDetection::RGBCameraCallback, this);
  //img_subscriber_ = it_.subscribe("zed/rgb/image_rect_color", 1, &SignDetection::RGBCameraCallback, this);

  std::string cascade_file;
  nh.param<std::string>("HaarCascadeFile", cascade_file, "");

  // Load in classifier
  sign_cascade.load(cascade_file);
  if( sign_cascade.empty() )
    {
        ROS_ERROR("ERROR LOADING CLASSIFIER");
    };
}

void expand_rect(cv::Rect& rect, cv::Size img_size) {
  // Helper function to expand rectangle in order to contain entire sign
  float percent_expand = 1.2;

  // Increase rect size and recenter using an offset
  cv::Size delta_size(rect.width * percent_expand, rect.height * percent_expand);
  cv::Point offset(delta_size.width / 2, delta_size.height / 2);
  rect += delta_size;
  rect -= offset;

  // Check if rect crosses image boundaries
  if (rect.x < 0) {
    rect.x = 0;
  } else {
    if (rect.x + rect.width > img_size.width) {
      rect.width = img_size.width - rect.x;
    }
  }

  if (rect.y < 0) {
    rect.y = 0;
  } else {
    if (rect.y + rect.height > img_size.width) {
      rect.height = img_size.height - rect.y;
    }
  }
}

// double get_median(cv::Mat img) {
//   // Helper function that quickly finds median value of an image
//   // Based on : https://stackoverflow.com/questions/30078756/super-fast-median-of-matrix-in-opencv-as-fast-as-matlab

//   // Copy image into a vector so we can sort
//   img = img.reshape(0,1);
//   std::vector<double> img_vector;
//   img.copyTo(img_vector);

//   // Set n-th element pivot to be at middle of array, the function sorts the array such that all
//   // elements before the pivot are smaller than it and all elements after the pivot are larger, but
//   // are not necessarily sorted
//   std::nth_element(img_vector.begin(), img_vector.begin() + img_vector.size() / 2, img_vector.end());
//   return img_vector[img_vector.size() / 2];
// }

void SignDetection::RGBCameraCallback(const sensor_msgs::ImageConstPtr& right_msg) {
  // Convert sensor_msgs::Image to a BGR8 cv::Mat 
  cv::Mat rgb_img = (cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::BGR8))->image;
  //cv::Mat depth_img = (cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO8))->image;
  cv::Mat right_img;
  cv::cvtColor(rgb_img, right_img, cv::COLOR_BGR2GRAY);
  cv::equalizeHist(right_img, right_img);

  // Apply classifier to grayscale image
  std::vector<cv::Rect> classifications;
  std::vector<int> reject_levels;
  std::vector<double> level_weights;
  //sign_cascade.detectMultiScale(right_img, classifications, reject_levels, level_weights, 1.05, 5, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(20, 20), true);
  sign_cascade.detectMultiScale(right_img, classifications, reject_levels, level_weights, 1.1, 4, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(20, 20), cv::Size(), true);
  
  cv::Rect best_bbox;
  if (classifications.empty()) {
    consecutive_frames = 0;
    // send "no sign" for message
  } else {
    // Keep bouding box with the highest level weight
    std::vector<int>::size_type best_index = 0;
    double best_weight = -100.0;
    for(std::vector<int>::size_type i = 0; i < classifications.size(); i++) {
      if (level_weights[i] > best_weight) {
        best_weight = level_weights[i];
        best_index = i;
      }
      ROS_INFO("bbox: (%4.2f y: %i, x: %i, w: %i, h: %i)", level_weights[i], classifications[i].y, classifications[i].x, classifications[i].width, classifications[i].height);
    }
    best_bbox = classifications[best_index];

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

  }

  cv::Mat arrow_img;
  ROS_INFO("Consecutive frames: %i", consecutive_frames);
  // Find arrow direction from sign
  if (consecutive_frames > 0) {
    // Expand bounding box to make sure it contains the arrow
    expand_rect(best_bbox, right_img.size());
    // add rect to img
    cv::rectangle(rgb_img, best_bbox, cv::Scalar(0, 0, 255));

    // Get direction of arrow
    arrow_img = CheckArrowDir(cv::Mat(right_img, best_bbox).clone());
  }

  


  // Add classifications to image
  // for (cv::Rect bbox : classifications) {
  //   cv::rectangle(rgb_img, bbox, cv::Scalar(255, 0, 0));
  // }

  // Publish so we can visualize in rviz
  cv_bridge::CvImage img_bridge_output;
  std_msgs::Header header; // empty header
  header.stamp = ros::Time::now(); // time
  img_bridge_output = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, arrow_img);
  //img_bridge_output = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, rgb_img);

  img_publisher_.publish(img_bridge_output.toImageMsg());
  ROS_INFO("IMAGE DATA PUBLISHED");
}

// void sortPoints(std::vector<cv::Point> &points) {
//   // Sorts 4 points in order: bot-left, top-left, bot-right, top-right
//   // Needed to map correct corners in perspective transform
//   struct SortX {
//     bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.x < pt2.x);}
//   } x_sorter;

//   struct SortY {
//       bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.y < pt2.y);}
//   } y_sorter;

//   // Sort by x-coordinate
//   std::sort(points.begin(), points.end(), x_sorter);

//   // Sort 2 left most and 2 right most by y-coordinate separately
//   std::sort(points.begin(), points.begin() + 1, y_sorter);
//   std::sort(points.begin() + 2, points.begin() + 3, y_sorter);
// }

cv::Mat SignDetection::CheckArrowDir(cv::Mat sign) {
  // Function returns the direction of the arrow within the givin sign
  // Summary: 
  // Apply canny to get outline of arrow and get it's contour
  // Fit an ellipse to the chosen contour to get info on it's orientation
  // If vertical, arrow is direction is straight
  // If horizontal, apply secondary algorithm to determine if left or right (explained below)

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
  
  // Get contour that is closest to the center (should be arrow)
  cv::Point2f image_center(sign.size().width / 2, sign.size().height / 2);
  double smallest_dist = std::max(sign.size().width, sign.size().height) * 2;
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

  // Create arrow matrix
  cv::Mat arrow = cv::Mat::zeros(sign.size(), CV_8UC1);
  cv::drawContours(arrow, contours, sd_index, cv::Scalar(255), CV_FILLED);

  cv::Mat arrow_bgr;
  cv::cvtColor(arrow, arrow_bgr, cv::COLOR_GRAY2BGR);

  // Fit an ellipse to the arrow and get some parameters from it
  cv::RotatedRect rect_fit = cv::fitEllipse(contours[sd_index]);
  
  float angle = rect_fit.angle;
  cv::Size2f rect_size = rect_fit.size;
  cv::Point2f rect_center = rect_fit.center;

  ROS_INFO("Angle: %f; w, h: (%f, %f)", angle, rect_size.width, rect_size.height);

  // If angle is vertical, conclude that arrow direction is straight
  // Angle ranges 0-180 deg from the vertival axis going CW
  if (angle < 45 || angle > 135) {
    // Sign is straight
    ROS_INFO("Straight");

  } else {
    // Sign is horizontal, need to apply secondary check, summary of check is:
    // Get a line perpendicular to the orientation of the arrow that passes through it's middle
    // Use this line to create two masks that split the arrow down the middle
    // Mask the arrow and count the number of pixels on each side
    // The side with the most pixels will be the side with the arrow head
    
    // Get slope of minor axis
    float slope = tan((angle) * M_PI / 180);

    // Define line lambda function that passes through rect_center
    auto get_y = [&](int x) { return slope * (x - rect_center.x) + rect_center.y; };
    auto get_x = [&](int y) { return 1 / slope * (y - rect_center.y) + rect_center.x; };

    // Find corrdinates for points that go through all four edges
    cv::Point left(0, get_y(0));
    cv::Point right(arrow.cols, get_y(arrow.cols));
    cv::Point top(get_x(0), 0);
    cv::Point bottom(get_x(arrow.rows), arrow.rows);

    ROS_INFO("L: (%i, %i), R: (%i, %i), T: (%i, %i), B: (%i, %i)", left.x, left.y, right.x, right.y, top.x, top.y, bottom.x, bottom.y);

    // Only keep the two points that make sense (other two will be outside the image)
    std::vector<cv::Point> edge_points1;
    std::vector<cv::Point> edge_points2;
    if (left.y >= 0 && left.y < arrow.rows) {
      edge_points1.push_back(left);
      edge_points1.push_back(cv::Point(0, 0));
      edge_points1.push_back(cv::Point(arrow.cols-1, 0));

      edge_points2.push_back(left);
      edge_points2.push_back(cv::Point(0, arrow.rows-1));
      edge_points2.push_back(cv::Point(arrow.cols-1, arrow.rows-1));
    }
    if (right.y >= 0 && right.y < arrow.rows) {
      edge_points1.push_back(right);
      edge_points2.push_back(right);
    }
    if (top.x >= 0 && top.x < arrow.cols) {
      
      
      

      edge_points1.push_back(bottom);
      edge_points1.push_back(cv::Point(0, arrow.rows-1));
      edge_points1.push_back(cv::Point(0, 0));
      edge_points1.push_back(top);


      
      
      

      edge_points2.push_back(bottom);
      edge_points2.push_back(cv::Point(arrow.cols-1, arrow.rows-1));
      edge_points2.push_back(cv::Point(arrow.cols-1, 0));
      edge_points2.push_back(top);
    }
    // if (bottom.x >= 0 && bottom.x < arrow.cols) {
    //   edge_points1.push_back(bottom);
    //   edge_points2.push_back(bottom);
    // }

    // Remove potential duplicates (if point is located in a corner)
    // if (edge_points.size() > 2) {
      
    // }

    // Define the other corners needed to create a polygon mask


    cv::Mat mask1 = cv::Mat::zeros(arrow.size(), CV_8U);
    cv::fillConvexPoly(mask1, edge_points1, cv::Scalar(255));

    // Draw masks on image with alpha blending
    double alpha = 0.5;
    cv::Mat overlay;

    arrow_bgr.copyTo(overlay);

    cv::fillConvexPoly(overlay, edge_points1, cv::Scalar(255, 0, 0));
    cv::fillConvexPoly(overlay, edge_points2, cv::Scalar(0, 255, 0));

    cv::addWeighted(overlay, alpha, arrow_bgr, 1 - alpha, 0, arrow_bgr);

    // Apply mask to arrow and count pixels
    cv::Mat mask2 = cv::Mat::zeros(arrow.size(), CV_8U);
    cv::Mat arrow1, arrow2;
    arrow.copyTo(arrow1, mask1);
    cv::bitwise_not(mask1, mask2);
    arrow.copyTo(arrow2, mask2);

    // Count pixels in arrow1 and arrow2
    int area1 = cv::countNonZero(arrow1);
    int area2 = cv::countNonZero(arrow2);

    int total_area = cv::countNonZero(arrow);

    ROS_INFO("Green: %i, Blue: %i, total: %i", area2, area1, total_area);

    // Add visuals
    // cv::line(arrow_bgr, top, bottom, cv::Scalar(255, 0, 0));
    // cv::circle(arrow_bgr, rect_center, 1, cv::Scalar(0, 0, 255));
    // cv::ellipse(arrow_bgr, rect_fit, cv::Scalar(0, 255, 0));

  }


  return arrow_bgr;
}