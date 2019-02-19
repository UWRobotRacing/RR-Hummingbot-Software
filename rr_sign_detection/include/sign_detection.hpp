/** @file sign_detection.hpp
 *  @author Andrew Jin (D29Jin)
 *  @breif Sign Detection Class header file
 *  @competition IARRC 2019
 */
#ifndef __SIGN_DETECTION_PROCESSOR_HPP
#define __SIGN_DETECTION_PROCESSOR_HPP

//ROS
#include <ros/ros.h>

//OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//LOCAL

class SignDetection
{
  private:

  public:
    SignDetection(ros::NodeHandle nh);
    ~SignDetection();

};

#endif //__SIGN_DETECTION_PROCESSOR_HPP