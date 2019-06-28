/** @file computer_vision_main.cpp
 *  @author Andrew Jin
 *  @competition IARRC 2019
 */


//OPENCV

//LOCAL
#include "computer_vision.hpp"
<<<<<<< HEAD
#include "lane_detection.hpp"
=======
#include "endline_detection.hpp"
>>>>>>> develop

/** @brief main file that starts the subscribers and calls spin
 */
int main(int argc, char** argv)
{
  //Node and image transport initialization
  ros::init(argc, argv, "rr_computer_vision");
  ROS_INFO("Initializing Computer vision node");
  ros::NodeHandle nh;

  ROS_INFO("Starting Computer Vision Node");
  
  ComputerVision computer_vision(nh);
  // EndlineDetection ec(nh);
  LaneDetection lane_detection(nh);

  while(ros::ok()){
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
