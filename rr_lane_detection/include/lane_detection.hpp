/** @file lane_detection_processor.hpp
 *  @author Matthew Post
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */
#ifndef __LANE_DETECTION_HPP
#define __LANE_DETECTION_HPP


//ROS
#include <ros/ros.h>

//OPENCV includes
#include <opencv2/opencv.hpp>

//LOCAL

class LaneDetection
{
  private:

  public:
    LaneDetection(ros::NodeHandle nh);
    ~LaneDetection();
};

#endif //__LANE_DETECTION_HPP