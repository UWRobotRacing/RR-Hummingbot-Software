/** @file computer_vision_main.cpp
 *  @author Andrew Jin
 *  @competition IARRC 2019
 */

//ROS
#include <ros/ros.h>

//OPENCV

//LOCAL
#include "computer_vision.hpp"
#include "lane_detection.hpp"

/** @brief main file that starts the subscribers and calls spin
 */
int main(int argc, char** argv)
{
  //Node and image transport initialization
  ros::init(argc, argv, "rr_computer_vision");
  ROS_INFO("Initializing Computer vision node");
  ros::NodeHandle nh;
  ros::Rate r(5);
  // Construct neccesarry objects
  ComputerVision computer_vision(nh);
  LaneDetection lane_detection(nh);

  while(ros::ok()){
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
