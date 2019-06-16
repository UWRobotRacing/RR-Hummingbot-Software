/* 
 *  @file sign_detection.cpp
 *  @brief Sign Detection Implementation Class
 *
 * Topics Subscribed:
 *   /rr_
 *
 * Service called:
 *   /
 *
 *  @author Martin Ethier (MartinEthier)
 *  @competition IARRC 2019
 */

//LOCAL
#include "sign_detection.hpp"

/*
 * @name SignDetection 
 * @brief Constructor
 * @param nh: ROS node handler
 * @return NONE
*/
SignDetection::SignDetection(ros::NodeHandle nh_) : it_(nh_) {
  InitializePublishers();
  InitializeSubscribers();
}

/*
 * @name ~SignDetection
 * @brief Destructor
 * @return NONE
*/
SignDetection::~SignDetection() {
}

void SignDetection::InitializePublishers() {
  test_publisher = nh_.advertise<sensor_msgs::Image>("/test_stitching", 1);
}

void SignDetection::InitializeSubscribers() {
  // need left image and depth map

  left_camera_subscriber_.subscribe(nh_, "/zed/left/image_rect_color", 1);
  depth_map_subscriber_.subscribe(nh_, "/zed/depth/depth_registered", 1);
  rgb_depth_sync_.reset(new RGB_DEPTH_SYNC(RGB_DEPTH_POLICY(0), left_camera_subscriber_, depth_map_subscriber_));
  rgb_depth_sync_->registerCallback(boost::bind(&ComputerVision::RGBDepthSyncCameraCallback, this, _1, _2));

  // left_header_subscriber_ = nh_.subscribe("/zed/left/image_rect_color", 0, &ComputerVision::LeftCameraCallback, this);
  // depth_header_subscriber_ = nh_.subscribe("depth/depth_registered", 0, &ComputerVision::DepthCameraCallback, this);
}

void ComputerVision::RGBDepthSyncCameraCallback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& depth_msg) {
  ROS_INFO("SYNCHRONIZED POLICY CALLBACK");

  // Convert sensor_msgs::Image to a BGR8 cv::Mat 
  cv::Mat left_img_bgr8 = (cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::BGR8))->image;
  cv::Mat depth_img_bgr8 = (cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::BGR8))->image;

  // Apply classifier to rgb image
  classifications = GetClassifications(left_img_bgr8);

  // Filter out classifications using depth map
  final_sign = FilterSigns(classifications, left_img_bgr8);



  // Find arrow direction from sign


  // Publish so we can visualize in rviz
  cv_bridge::CvImage img_bridge_output;
  std_msgs::Header header; // empty header
  header.stamp = ros::Time::now(); // time
  img_bridge_output = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, result);

  test_publisher.publish(img_bridge_output.toImageMsg());
  ROS_INFO("SYNCHRONIZED DATA PUBLUISHED");
}


void SignDetection::GetClassifications(cv::Mat img) {
    //returns bounding boxes of classified signs

    std::vector<cv::Rect> signs;


    sign_cascade.detectMultiScale( img, signs, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );


    return signs
}

void SignDetection::FilterSigns(std::vector<cv::Rect> signs, cv::Mat img) {
  //

  for ( size_t i = 0; i < signs.size(); i++ ) {


    cv::Mat signROI = frame_gray( signs[i] );

  }


}

void SignDetection::CheckArrowDir(const cv::Mat& traffic_sign_image) {

}