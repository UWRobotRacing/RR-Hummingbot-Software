/**
 * @file main.cpp
 * @author Toni Ogunmade(oluwatoni)
 * @author Ji Min Kim
 * @competition IARRC 2018
 */

// ROS
#include <ros/ros.h>

// LOCAL
#include "traffic_light.hpp"

/**
 * @brief initializes and starts the traffic light detection node
 * @return int
 */
int main(int argc, char **argv)
{
  // Initiate trafficLight Node
  ros::init(argc, argv, "trafficLightNode");
  ros::NodeHandle nh_;
  TrafficLight traffic_light(nh_);

  ROS_INFO("Traffic Light Enabled");
  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
