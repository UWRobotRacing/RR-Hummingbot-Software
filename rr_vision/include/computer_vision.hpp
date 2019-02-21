/** @file computer_vision.hpp
 *  @author Matthew Post
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */
#ifndef COMPUTER_VISION
#define COMPUTER_VISION


//ROS
#include <ros/ros.h>

//OPENCV includes
#include <opencv2/opencv.hpp>

//LOCAL

class ComputerVision
{
  private:

  public:
    ComputerVision(ros::NodeHandle nh);
    ~ComputerVision();
};

#endif // COMPUTER_VISION