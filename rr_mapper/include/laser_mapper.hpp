/** @file laser_mapper.hpp
 *  @author Andrew Jin (D29Jin)
 *  @competition IARRC 2019
 */

#ifndef LASERMAPPER_H
#define LASERMAPPER_H

// ROS headers
#include <ros/ros.h>

/*
 * Callback Class for laser to Occumpancy Grid Format/OpenCV Format
 */

class LaserMapper
{
  public:
    LaserMapper(ros::NodeHandle nh);
    ~LaserMapper();

  private:
};

#endif  // LASERMAPPER_H
