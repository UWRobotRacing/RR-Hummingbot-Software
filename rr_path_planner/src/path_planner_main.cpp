/** @file path_planner_main.cpp
 *  @author Andrew Jin
 *  @competition IARRC 2019
 */

#include <ros/ros.h>

#include <path_planner.h>

/** @brief starts the path_planner node
 *  @return NONE
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rr_pathplanner");
  ros::NodeHandle n;
  ROS_INFO("Path Planner Initalized");
  PathPlanner path_planner;
  ros::spin();
  return 0;
}
