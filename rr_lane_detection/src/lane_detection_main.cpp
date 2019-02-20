/** @file lane_detection_main.cpp
 *  @author Andrew Jin
 *  @competition IARRC 2019
 */

//ROS
#include <ros/ros.h>

//OPENCV

//LOCAL
#include "lane_detection.hpp"

/** @brief main file that starts the subscribers and calls spin
 */
int main(int argc, char** argv)
{
  //Node and image transport initialization
  ros::init(argc, argv, "lane_detection");
  ros::NodeHandle nh;
  ROS_INFO("Starting lane detection");
  LaneDetection lane_detection(nh);

  ros::spin();
  return 0;
}