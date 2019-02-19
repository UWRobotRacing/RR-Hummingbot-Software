/** @file laser_mapper.hpp
 *  @author Andrew Jin (D29Jin)
 *  @competition IARRC 2019
 */

#ifndef LASERMAPPER_H
#define LASERMAPPER_H

// CPP
#include <stdio.h>
#include <math.h>
#include <vector>
#include <string>

// ROS headers
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

/*
 *  Callback Class for laser to Occumpancy Grid Format/OpenCV Format
 */

class LaserMapper
{
  public:
    LaserMapper(ros::NodeHandle nh);
    ~LaserMapper();

  private:
};

#endif  // LASERMAPPER_H
