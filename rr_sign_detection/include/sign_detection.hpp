/** @file sign_detection.hpp
 *  @author Andrew Jin (D29Jin)
 *  @breif Sign Detection Class header file
 *  @competition IARRC 2019
 */
#ifndef SIGN_DETECTION_PROCESSOR_HPP
#define SIGN_DETECTION_PROCESSOR_HPP

//ROS
#include <ros/ros.h>

class SignDetection
{
  private:

  public:
    SignDetection(ros::NodeHandle nh);
    ~SignDetection();

};

#endif //SIGN_DETECTION_PROCESSOR_HPP