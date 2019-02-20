/** @file main.cpp
 *  @author Andrew Jin
 *  @breif Sign Detection main to ensure spin
 *  @competition IARRC 2019
 */

//LOCAL
#include "sign_detection.hpp"

/** @brief main file that starts the subscribers and calls spin
 */
int main(int argc, char** argv)
{
  //Node and image transport initialization
  ros::init(argc, argv, "sign_detection");
  ros::NodeHandle nh;
  ROS_INFO("Starting lane detection");
  SignDetection sign_detection(nh);

  ros::spin();
  return 0;
}