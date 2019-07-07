/** @file computer_vision_main.cpp
 *  @author Andrew Jin
 *  @competition IARRC 2019
 */

//LOCAL
#include "computer_vision.hpp"
#include "traffic_light.hpp"
#include "lane_detection.hpp"
#include "endline_detection.hpp"

/** @brief main file that starts the subscribers and calls spin
 */
int main(int argc, char** argv)
{
  //Node and image transport initialization
  ros::init(argc, argv, "rr_computer_vision");
  ROS_INFO("Initializing Computer Vision Node");
  ros::NodeHandle nh;
  
  // ComputerVision computer_vision(nh);
  // TrafficLightDetection traffic_light_detection(nh);
  // EndlineDetection endline_detection(nh);
  LaneDetection lane_detection(nh);

  ros::spin();
  return 0;
}