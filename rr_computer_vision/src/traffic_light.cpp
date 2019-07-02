/**
 * @file raffic_light.cpp
 * @brief Traffic Light cpp File
 * @author Yuchi(Allan) Zhao
 * @author Waleed Ahmed
 * @competition IARRC 2019
 */

#include "traffic_light.hpp"
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// #include <sensor_msgs/Image.h>
#include <ros/console.h>

TrafficLightDetection::TrafficLightDetection(ros::NodeHandle nh) : it_(nh_), red_light_detected_(false), default_ratio_(0),
redLightCounter_(0), greenLightCounter_(0)  {
  // test_subscriber = it_.subscribe("/zed/left/image_rect_color", 1, &TrafficLightProcessor::TrafficLightImageCallback, this);
  // test_publisher = it_.advertise("/test_traffic_light", 1);
  client_ = nh_.serviceClient<std_srvs::Empty>("/Supervisor/start_race");
  SetParams();
  // Set up detector with params
  detector_ = cv::SimpleBlobDetector::create(params);        
}

void TrafficLightDetection::SetParams() {
        // Filter by Area
        params.filterByArea = true;
        params.minArea = 300;
        
        // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = 0.6;
        
        // Filter by Convexity
        params.filterByConvexity = true;
        params.minConvexity = 0.6;
        
        // Filter by Inertia
        params.filterByInertia = false;
        //params.minInertiaRatio = 0.01;

        // Filter white
        params.blobColor=255;
}

void TrafficLightDetection::FindBoundRect(const int maxAreaIndex, cv::Mat& crop_img, const cv::Mat& threshold_img, std::vector<cv::KeyPoint>& keypoints){
  // the center and diameter of the blob
  int blob_x=(int)keypoints[maxAreaIndex].pt.x;
  int blob_y=(int)keypoints[maxAreaIndex].pt.y;
  int blob_diameter=(int)keypoints[maxAreaIndex].size;
  int blob_radius=(int) blob_diameter/2;
  
  // create a reatangle around the blob
  cv::Rect firstRect (blob_x-blob_radius, blob_y-blob_radius, blob_diameter, blob_diameter);   
  crop_img=threshold_img(firstRect);
  boundRect_=firstRect;
}

void TrafficLightDetection::RedLightDetection(const cv::Mat& threshold_img, int& maxAreaIndex, double& new_ratio){
  // Detect blobs.
  std::vector<cv::KeyPoint> keypoints;
  detector_->detect( threshold_img, keypoints);
  int numBlob=keypoints.size();
  cv::Mat crop_img;

  if(numBlob > 0){
    // find the largest blob if there are more than 2 blobs
    if(numBlob > 1){
      for (int i = 0; i < numBlob; i++)
      {
        if (keypoints[maxAreaIndex].size < keypoints[i+1].size)
        {
          maxAreaIndex = i+1;
        }
      }
    }

    FindBoundRect(maxAreaIndex, crop_img, threshold_img, keypoints);
    // count pixel in the rectangle
    int red_Pixel_Counter=cv::countNonZero(crop_img);
    int total_pixel=crop_img.total();
    
    if(redLightCounter_ == 0){
      default_ratio_=(double)red_Pixel_Counter/total_pixel;
      // ROS_INFO("default ratio: %f", default_ratio);
      redLightCounter_++;
    } else {
      new_ratio=(double)red_Pixel_Counter/total_pixel; 

      // red light buffer             
      if((new_ratio+0.1)>= default_ratio_){
        redLightCounter_++;
      }
    }
    
    if(redLightCounter_>=10){
      ROS_INFO("Red light detected!");
      red_light_detected_=true;
      redLightCounter_=0;
    }
  }
}

void TrafficLightDetection::ColorFilter(const cv::Mat& hsv_img, cv::Mat& threshold_img){
  // Filter red
  cv::inRange(hsv_img,cv::Scalar(0, 90, 155), cv::Scalar(15, 255, 255), threshold_img);
  cv::GaussianBlur(threshold_img, threshold_img, cv::Size(7,7), 0, 0);
}

void TrafficLightDetection::TrafficLightImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat traffic_light_image, hsv_img, threshold_img, blur_img, im_with_keypoints;
  cv_bridge::CvImagePtr cv_ptr;
  std_srvs::Trigger srv;
  double new_ratio=0;
  int maxAreaIndex = 0;

  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    traffic_light_image = cv_ptr->image;
    cv::cvtColor(traffic_light_image, hsv_img, CV_BGR2HSV);

    ColorFilter(hsv_img, threshold_img);
    // PUBLISH and visualize in rviz   
    // cv_bridge::CvImage img_bridge_output;
    // std_msgs::Header header;
    // header.stamp=ros::Time::now();
    // img_bridge_output=cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, threshold_img);
    // test_publisher.publish(img_bridge_output.toImageMsg());

    if(red_light_detected_==false){
        RedLightDetection(threshold_img, maxAreaIndex, new_ratio);
    }else {
      //crop the existing rectangle on each new frame and count pixel
      cv::Mat crop_img=threshold_img(boundRect_);
      int red_Pixel_Counter=cv::countNonZero(crop_img);
      int total_pixel=crop_img.total();
      new_ratio= (double)red_Pixel_Counter/total_pixel;

      if((new_ratio+0.1)>= default_ratio_){
        ROS_INFO("Red light is still there!");
      }else{
        // green light buffer
        greenLightCounter_++;
        if(greenLightCounter_>= 5){
          ROS_INFO("Green light detected!");
          if (client_.call(srv)){
            ros::shutdown();
          }
        }
      }
    }

  }catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}