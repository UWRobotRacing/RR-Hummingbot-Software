/** @file computer_vision_main.cpp
 *  @author Andrew Jin
 *  @competition IARRC 2019
 */

//ROS
#include <ros/ros.h>

//OPENCV

//LOCAL
#include "computer_vision.hpp"
#include "endline_detection.hpp"

/** @brief main file that starts the subscribers and calls spin
 */
int main(int argc, char** argv)
{
  //Node and image transport initialization
  ros::init(argc, argv, "rr_computer_vision");
  ros::NodeHandle nh;
  ROS_INFO("Starting Computer Vision Node");

  ComputerVision computer_vision(nh);
  EndlineCounter ec(nh);

  ros::spin();
  return 0;
}