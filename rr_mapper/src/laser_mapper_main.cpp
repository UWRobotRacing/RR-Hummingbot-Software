/** @file main.cpp
 * 
 * This function takes in the scan messages, and translate it into occupancy
 * grid. If vision occupancy grid is available, and the option is turned on,
 * this mapper will only publish messages when new vision data is availablei.
 * 
 * If enable is off, then nothing is published! Resets the laser map.
 * Always assumes initial vehicle position is at (0,0) = mid bottom of the image
 * Grid size changes with Vision Occupancy Grid
 * Parameters are loaded in the robotracing trajectory launch node. 
 * The node keeps two maps, vision and laser. If vision node is not available
 * Vision node is not updated with obstacles (empty 0s)
 * 
 *  @author Jungwook Lee
 *  @author Sirui Song
 *  @author Raymond Kuo
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */

#include <ros/ros.h>

#include "laser_mapper.hpp"

/** @brief starts the laser_mapper node
 *
 *  nothing happens until enable is true
 *
 *  @return NONE
 */ 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rr_mapper");
  ros::NodeHandle nh;
  LaserMapper laserProcessor(nh);

  ROS_INFO("Mapper: Laser Node Initialized");
  ros::Rate r(30);
  while(ros::ok())
  {
    //This will ensure that at least one message will be flushed in before publishing
    ros::spinOnce();
    laserProcessor.PublishMap();
    r.sleep();
  }

  return 0;
}
