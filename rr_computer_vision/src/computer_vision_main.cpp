/** @file computer_vision_main.cpp
 *  @author Andrew Jin
 *  @competition IARRC 2019
 */


//OPENCV

//LOCAL
#include "computer_vision.hpp"
#include "endline_detection.hpp"
#include "sign_detection.hpp"

/** @brief main file that starts the subscribers and calls spin
 */
int main(int argc, char** argv)
{
  //Node and image transport initialization
  ros::init(argc, argv, "rr_computer_vision");
  ros::NodeHandle nh;
  ROS_INFO("Starting Computer Vision Node");
  
  ComputerVision computer_vision(nh);
  // EndlineDetection ec(nh);
  SignDetection sd(nh);

  ros::spin();
  return 0;
}