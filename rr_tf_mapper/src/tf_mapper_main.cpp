/** @file tf_mapper_main.cpp
 *  @author Andrew Jin
 *  @competition IARRC 2019
 */

#include <ros/ros.h>

#include <tf_mapper.h>

/** @brief starts the tf_mapper node
 *  @return NONE
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_mapper");
  ros::NodeHandle n;
  ROS_INFO("TF Mapper Initialized");
  TFMapper tf_mapper;
  return 0;
}
