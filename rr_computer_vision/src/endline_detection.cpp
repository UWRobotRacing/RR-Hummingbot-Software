/** @file endline_detection.cpp
 *  endline detection function implementation && main included at the end
 *
 * Topics Subscribed:
 *   /rr_vehicle/front_facing_cam/image_raw
 *
 * Service called:
 *   /Supervisor/count_lap         --used by supervisor node
 * @author Yuchi(Allan) Zhao
*/


#include "endline_detection.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

//constructor
EndlineCounter::EndlineCounter(ros::NodeHandle nh) : it_(nh_) 
{
  detection_status_ = false;
  client_ = nh_.serviceClient<std_srvs::Trigger>("/Supervisor/count_lap");
  
  test_subscriber = it_.subscribe("/zed/rgb/image_rect_color", 1, &EndlineCounter::ImgCb, this);
  test_publisher = it_.advertise("/test_endline", 1);
}

bool EndlineCounter::compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
  double i = fabs( cv::contourArea(cv::Mat(contour1)) );
  double j = fabs( cv::contourArea(cv::Mat(contour2)) );
  return ( i < j );
}

//callback to handle detection
void EndlineCounter::ImgCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat init_img, hsv_img, mag_img, blur_img;
  cv_bridge::CvImagePtr cv_ptr;
  std_srvs::Trigger srv;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<cv::Point> approx;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //filter for magenta
    init_img = cv_ptr->image;
    cv::cvtColor(init_img, hsv_img, CV_BGR2HSV);
    cv::inRange(hsv_img, cv::Scalar(LowHue, LowSat,LowVal), cv::Scalar(HighHue, HighSat,HighVal), mag_img);
    cv::GaussianBlur(mag_img, mag_img, cv::Size(7,7), 0, 0);

    // PUBLISH and visualize in rviz   
    cv_bridge::CvImage img_bridge_output;
    std_msgs::Header header;
    header.stamp=ros::Time::now();
    img_bridge_output=cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, mag_img);
    test_publisher.publish(img_bridge_output.toImageMsg());

    // Find Contours
    cv::findContours(mag_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    int conSize = contours.size();
    ROS_INFO("Number of contours: %d", conSize);

    // Sort contours by area (smallest to largest)
    std::sort(contours.begin(), contours.end(), compareContourAreas);
    
    // contours[contours.size()-1] will be the contour with the largest area
    double maxArea=cv::contourArea(cv::Mat(contours[contours.size()-1]));
    ROS_INFO("Max contour area: %f", maxArea);

    cv::approxPolyDP(cv::Mat(contours[contours.size()-1]), approx, 0.1*arcLength(cv::Mat(contours[contours.size()-1]), true), true);
    int polyLength = approx.size();
    ROS_INFO("Polygon length: %d", polyLength);

    if (approx.size()==4)
    {
        //bound with a rectangle
        //std::vector<cv::Rect> boundRect;
        cv::Rect boundRect = cv::boundingRect(cv::Mat(contours[contours.size()-1]));
  
        //check the ratio  if it is an endline
        if ((boundRect.width/boundRect.height)>22.5)
        {
          ROS_INFO("DETECTED");
          //check and change status
          if (!detection_status_)
          {
            detection_status_ = true;
          }
        }
        else
        {
          //not detected or not an endline
          //if the endline is not detected, NOTHING CHANGED!!! Keep looping.
          //check if the endline is no longer detected
          if (detection_status_ )
          {
            ROS_INFO("Ready to stop!!!");
             //make service call
            if (client_.call(srv))
            {
              if (srv.response.success)
              {
                ROS_INFO("SUCCESS");
                ros::shutdown();
              }
            }
          }
        }
    } 
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}


// try
  // {
  //   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
  //   //filter for magenta
  //   ROS_INFO("after find contour");
  //   init_img = cv_ptr->image;
  //   cv::cvtColor(init_img, hsv_img, CV_BGR2HSV);
  //   cv::inRange(hsv_img, cv::Scalar(LowHue, LowSat,LowVal), cv::Scalar(HighHue, HighSat,HighVal), mag_img);
  //   cv::GaussianBlur(mag_img, mag_img, cv::Size(7,7), 0, 0);

  //   // PUBLISH and visualize in rviz   
  //   ROS_INFO("after find contour");
  //   cv_bridge::CvImage img_bridge_output;
  //   std_msgs::Header header;
  //   header.stamp=ros::Time::now();
  //   img_bridge_output=cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, mag_img);
  //   test_publisher.publish(img_bridge_output.toImageMsg());

  //   //contour 
  //   ROS_INFO("after find contour");
  //   cv::findContours(mag_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  //   for (int i = 0; i < contours.size(); i++)
  //   {
  //     ROS_INFO("in loop");
  //     cv::approxPolyDP(cv::Mat(contours[i]), approx, 0.1*arcLength(cv::Mat(contours[i]), true), true);

  //     if (approx.size()==4)
  //     {
  //       ROS_INFO("in four");
  //       //bound with a rectangle
  //       //std::vector<cv::Rect> boundRect;
  //       cv::Rect boundRect = cv::boundingRect(cv::Mat(contours[i]));
        
  //       //check the ratio  if it is an endline
  //       if ((boundRect.width/boundRect.height)>22.5)
  //       {
  //         ROS_INFO("DETECTED");
  //         //check and change status
  //         if (!detection_status_)
  //         {
  //           detection_status_ = true;
  //         }
  //         //get out of looping
  //         break;
  //       }
  //       else
  //       {
  //         //not detected or not an endline
  //         //if the endline is not detected, NOTHING CHANGED!!! Keep looping.
  //         //check if the endline is no longer detected
  //         if (detection_status_ )
  //         {
  //           ROS_INFO("Ready to stop!!!");
  //            //make service call
  //           if (client_.call(srv))
  //           {
  //             if (srv.response.success)
  //             {
  //               ROS_INFO("SUCCESS");
  //               ros::shutdown();
  //             }
  //           }
  //         }
  //       }
  //     } 
  //   }
  // }