// Notes:
// cv::Rect bbox;
// bbox.x, bbox.y, bbox.width, bbox.height (x, y is top-left corner)



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

/*
 * @name SignDetection 
 * @brief Constructor
 * @param nh: ROS node handler
 * @return NONE
*/
SignDetection::SignDetection(ros::NodeHandle nh_) : it_(nh_) {
  InitializePublishers();
  InitializeSubscribers();

  // Load in classifier
  if ( !sign_cascade.load( cascade_name ) ) { ROS_INFO("ERROR LOADING CLASSIFIER"); };
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
  cv::Mat left_img = (cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::BGR8))->image;
  cv::Mat depth_img = (cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO8))->image;

  cv::cvtColor(left_img, left_img, cv::COLOR_BGR2GRAY);
  cv::equalizeHist(left_img, left_img);

  // Apply classifier to grayscale image
  std::vector<cv::Rect> classifications;
  sign_cascade.detectMultiScale(left_img, classifications, 1.1, 3, 0|cv::CV_HAAR_SCALE_IMAGE, cv::Size(20, 20));

  // Filter out classifications using depth map
  if (classifications.empty()) {
    
  } else {

  }
  cv::Rect final_sign;
  final_sign = FilterSigns(classifications, left_img, depth_img);

  // Find arrow direction from sign
  int direction;
  direction = CheckArrowDir(cv::Mat(left_img, final_sign).clone());

  // Publish so we can visualize in rviz
  cv_bridge::CvImage img_bridge_output;
  std_msgs::Header header; // empty header
  header.stamp = ros::Time::now(); // time
  img_bridge_output = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, result);

  test_publisher.publish(img_bridge_output.toImageMsg());
  ROS_INFO("SYNCHRONIZED DATA PUBLISHED");
}

cv::Rect SignDetection::FilterSigns(std::vector<cv::Rect> signs, cv::Mat img) {
  // if depth map is not reliable, try the following:
  // for each bbox from classifier
  // convert bbox region to hsv
  // check for low saturation (grayscale) because sign is black n white
  // only keep signs with high % of low saturation pixels



  for ( size_t i = 0; i < signs.size(); i++ ) {


    cv::Mat signROI = frame_gray( signs[i] );

  }


}

struct SortX {
    bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.x < pt2.x);}
} x_sorter;

struct SortY {
    bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.y < pt2.y);}
} y_sorter;

void sortPoints(std::vector<cv::Point> &points) {
  // Sorts 4 points in order: bot-left, top-left, bot-right, top-right
  // Needed to map correct corners in perspective transform


  // Sort by x-coordinate
  std::sort(points.begin(), points.end(), x_sorter);

  // Sort 2 left most and 2 right most by y-coordinate separately
  std::sort(points.begin(), points.begin() + 1, y_sorter);
  std::sort(points.begin() + 2, points.begin() + 3, y_sorter);
}

int SignDetection::CheckArrowDir(cv::Mat sign) {
  // steps:
  // fit 4 sided polygon to sign
  // transform to a square
  // apply arrow direction algorithm to sign

  // Convert to binary and invert
  cv::Mat sign_bw_inv;
  cv::GaussianBlur(sign, sign_bw_inv, cv::Size(3, 3), 0, 0);
  cv::threshold(sign_bw_inv, sign_bw_inv, 0, 255, cv::CV_THRESH_BINARY_INV | cv::CV_THRESH_OTSU);

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(sign_bw_inv, contours, hierarchy, cv::CV_RETR_TREE, cv::CV_CHAIN_APPROX_SIMPLE);

  // Get largest contour
  int largest_area = 0;
  int largest_cnt_index = 0;
  for (int i = 0; i < contours.size(); i++) {
    double area = cv::contourArea(contours[i])
    if (area > largest_area) {
      largest_area = area;
      largest_cnt_index = i;
    }
  }

  // Approximate sign with a 4-sided polygon
  std::vector<cv::Point> approx;
  double d = 0;
  do
  {
      d++;
      cv::approxPolyDP(contours[largest_cnt_index], approx, d, true);
  }
  while (approx.size() > 4);

  // Define target for warp transform (square)
  int dst_size = std::max(sign.rows, sign.cols);
  std::vector<cv::Point> dst_points;
  dst_points.push_back(cv::Point(0, dst_size));
  dst_points.push_back(cv::Point(0, 0));
  dst_points.push_back(cv::Point(dst_size, dst_size));
  dst_points.push_back(cv::Point(dst_size, 0));
  
  // Sort points in same order as dst_points
  sortPoints(approx);

  // Apply perspective transform to square out the sign
  cv::Mat sign_bw = cv::Mat(sign_bw_inv.size(), sign_bw_inv.type())
  cv::bitwise_not(sign_bw_inv, sign_bw);
  cv::Mat square_img;
  //dst_img.create(approx.size(), approx.type());
  cv::Mat transform_mat = cv::getPerspectiveTransform(approx, dst_points)
  cv::warpPerspective(sign_bw, square_img, transform_mat, cv::Size(dst_size, dst_size), cv::INTER_NEAREST);

  // Grab largest contour (should be the arrow in the sign)
  std::vector<std::vector<cv::Point>> arrow_contours;
  std::vector<cv::Vec4i> arrow_hierarchy;
  cv::findContours(square_img, arrow_contours, arrow_hierarchy, cv::CV_RETR_TREE, cv::CV_CHAIN_APPROX_SIMPLE);
  
  // Get largest contour
  int largest_area = 0;
  int largest_cnt_index = 0;
  for (int i = 0; i < arrow_contours.size(); i++) {
    double area = cv::contourArea(arrow_contours[i])
    if (area > largest_area) {
      largest_area = area;
      largest_cnt_index = i;
    }
  }
  
  // Approximate arrow with a 7-sided polygon
  // https://stackoverflow.com/questions/13028961/how-to-force-approxpolydp-to-return-only-the-best-4-corners-opencv-2-4-2
  std::vector<cv::Point> approx;
  double d = 0;
  do
  {
      d++;
      cv::approxPolyDP(contours[largest_cnt_index], approx, d, true);
  }
  while (approx.size() > 7);












}