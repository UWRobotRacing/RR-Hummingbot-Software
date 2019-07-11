/** @file endline_detection.cpp
 *  @brief Magenta endline detection
 *  @author Waleed Ahmed (w29ahmed)
 *  @author Yuchi(Allan) Zhao
 *  @competition IARRC 2019
*/

// Standard includes
#include <vector>

// Local includes
#include "rr_topic_names.hpp"
#include "endline_detection.hpp"
#include "thresholding_values.hpp"

// ROS includes
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>

// OpenCV includes
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// Constructor
EndlineDetection::EndlineDetection(ros::NodeHandle nh) : it_(nh_), detection_status_(false), endline_counter_(0)
{
  client_ = nh_.serviceClient<std_srvs::Empty>(rr_supervisor::count_lap_service);
  img_subscriber_ = it_.subscribe(rr_sensor_topics::zed_left, 1, &EndlineDetection::EndlineImgCallback, this);
  // test_publisher_ = it_.advertise("/test_endline", 1);

  // Get hsv thresholding values based on the weather conditions
  int weather_condition;
  nh_.param<int>("WeatherCondition", weather_condition, 1);

  switch (weather_condition) {
    case 0:
      hsv_lower_bounds_ = endline_detection::overcast_hsv_lower_bounds;
      hsv_upper_bounds_ = endline_detection::overcast_hsv_upper_bounds;
      break;
    case 1:
      hsv_lower_bounds_ = endline_detection::sunny_hsv_lower_bounds;
      hsv_upper_bounds_ = endline_detection::sunny_hsv_upper_bounds;
      break;
    case 2:
      hsv_lower_bounds_ = endline_detection::sun_in_image_hsv_lower_bounds;
      hsv_upper_bounds_ = endline_detection::sun_in_image_hsv_upper_bounds;
      break;
    case 3:
      hsv_lower_bounds_ = endline_detection::indoor_hsv_lower_bounds;
      hsv_upper_bounds_ = endline_detection::indoor_hsv_upper_bounds;
      break;
  }
}

// Function used to sort by contour area
bool EndlineDetection::compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
  double i = fabs( cv::contourArea(cv::Mat(contour1)) );
  double j = fabs( cv::contourArea(cv::Mat(contour2)) );
  return ( i < j );
}

// Callback to detect endline
void EndlineDetection::EndlineImgCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat init_img, hsv_img, mag_img, blur_img;
  cv_bridge::CvImagePtr cv_ptr;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<cv::Point> approx;
  std_srvs::Empty srv;

  try
  {
    // Convert ros sensor image to a cv matrix
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    init_img = cv_ptr->image;

    // Colour thresholding to filter out magenta (colour of the endline)
    cv::cvtColor(init_img, hsv_img, CV_BGR2HSV);
    cv::inRange(hsv_img, hsv_lower_bounds_, hsv_upper_bounds_, mag_img);
    cv::GaussianBlur(mag_img, mag_img, cv::Size(7,7), 0, 0);

    // Publish thresholded image (For testing purposes)
    // cv_bridge::CvImage img_bridge_output;
    // std_msgs::Header header;
    // header.stamp=ros::Time::now();
    // img_bridge_output=cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, mag_img);
    // test_publisher_.publish(img_bridge_output.toImageMsg());

    // Find Contours in thersholded image
    cv::findContours(mag_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    int num_contours = contours.size();
    // ROS_INFO("Number of contours: %d", num_contours);

    // Sort contours by area (smallest to largest)
    std::sort(contours.begin(), contours.end(), compareContourAreas);

    // Extract the max contour area
    double maxArea;
    if (num_contours == 0)
    {
      maxArea = 0.00;
    }
    else
    {
      // contours[contours.size()-1] will be the contour with the largest area
      maxArea = cv::contourArea(cv::Mat(contours[contours.size()-1]));
    }
    // ROS_INFO("Max contour area: %f", maxArea);

    // For the endline to be considered 'detected', we wait for atleast 10 frames
    // in a row with a high enough max contour size. 1500 was used as the max contour
    // threshold as that is what the area of the contour is when the endline first comes into
    // view. For the endline to be gone, we need 10 frames in a row below that 1500 threshold.
    // This counter approach ensures no false positives arise from noisy frames
    if (!detection_status_)
    {
      if (maxArea > contour_area_cutoff_ && endline_counter_ < 10)
      {
        endline_counter_++;
      }
      else
      {
        endline_counter_ = 0;
      }

      if (endline_counter_ == 10)
      {
        ROS_INFO("Endline detected!");
        detection_status_ = true;
        endline_counter_ = 0;
      }
    }
    else 
    {
      if (maxArea < contour_area_cutoff_ && endline_counter_ < 10)
      {
        endline_counter_++;
      }
      else
      {
        endline_counter_ = 0;
      }

      if (endline_counter_ == 10)
      {
        detection_status_ = false;
        endline_counter_ = 0;

        // Make service call
        if (client_.call(srv))
        {
          // Shutdown all ROS clients, subscribers, and publishers
          client_.shutdown();
          img_subscriber_.shutdown();
          // test_publisher_.shutdown();
        }
      }
    }
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}