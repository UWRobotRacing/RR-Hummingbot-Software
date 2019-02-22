/** @file computer_vision.cpp
 *  @author Andrew Jin
 *  @competition IARRC 2019
 */

#include "computer_vision.hpp"

/*
 * @name ComputerVision 
 * @brief Constructor
 * @param nh: ROS node handler
*/
ComputerVision::ComputerVision(ros::NodeHandle nh) {

}

/*
 * @name ~ComputerVision
 * @brief Destructor
*/
ComputerVision::~ComputerVision() {

}

/*
 * @name InitializeSubscribers
 * @brief 
*/
void ComputerVision::InitializeSubscribers() {
    rgb_camera_subscriber_ = nh_.subscribe("rgb/image_rect_color", 0, &ComputerVision::RGBCameraCallback, this);
    left_camera_subscriber_ = nh_.subscribe("left/image_rect_color", 0, &ComputerVision::LeftCameraCallback, this);
    right_camera_subscriber_ = nh_.subscribe("right/image_rect_color", 0, &ComputerVision::RightCameraCallback, this);
    depth_camera_subscriber_ = nh_.subscribe("depth/depth_registered", 0, &ComputerVision::DepthCameraCallback, this);
}

void ComputerVision::RGBCameraCallback(const sensor_msgs::Image& msg){

}

void ComputerVision::LeftCameraCallback(const sensor_msgs::Image& msg){
    
}

void ComputerVision::RightCameraCallback(const sensor_msgs::Image& msg){
    
}

void ComputerVision::DepthCameraCallback(const sensor_msgs::Image& msg){
    
}