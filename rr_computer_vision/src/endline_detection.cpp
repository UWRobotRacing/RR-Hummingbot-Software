/** @file endline_detection.cpp
 *  endline detection function implementation && main included at the end
 *
 * Topics Subscribed:
 *   /rr_vehicle/front_facing_cam/image_raw
 *
 * Service called:
 *   /Supervisor/count_lap         --used by supervisor node
 *
 * @author Angela Gu (angegu)    
 * @author Toni Ogunmade (oluwatoni)
 * @author Yuchi(Allan) Zhao
 */


#include "endline_detection.hpp"

//constructor
EndlineCounter::EndlineCounter(ros::NodeHandle nh) : it_(nh_) 
{
  detection_status_ = false;
  // hysteresis_counter_ = 0;
  // hysteresis_constant_ = 2;
  client_ = nh_.serviceClient<std_srvs::Trigger>("/Supervisor/count_lap");
  
  test_subscriber = it_.subscribe("/zed/rgb/image_rect_color", 1, &EndlineCounter::ImgCb, this);
  test_publisher = it_.advertise("/test_endline", 1);
}

//callback to handle detection
void EndlineCounter::ImgCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat init_img, hsv_img, mag_img, blur_img;
  cv_bridge::CvImagePtr cv_ptr;
  std_srvs::Trigger srv;
  cv::vector<vector<Point> > contours;
  cv::vector<Vec4i> hierarchy;
  cv::vector<Point> approx;

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

    //contour 
    cv::findContours( mag_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    for (int i = 0; i < contours.size(); i++){
      cv::approxPolyDP(Mat(contours[i]), approx, 0.01*arcLength(Mat(contours[i], true),true)
      if (approx.size()==4){
        //bound with a rectangle
        cv::vector<Rect> boundRect;
        boundRect = cv::boundingRect( Mat(contours[i]));
        
        //check the ratio  if it is an endline
        if((boundRect.width/boundRect.height)>22.5){
          ROS_INFO("DETECTED");
          //check and change status
          if (!detection_status_){
            detection_status_ = true;
          }
          //get out of looping
          break;
        }else{     //not detected or not an endline
          //if the endline is not detected, NOTHING CHANGED!!! Keep looping.
          //check if the endline is no longer detected
          if(detection_status_ ){
            ROS_INFO("Ready to stop!!!");
             //make service call
            if (client_.call(srv)){
              if (srv.response.success){
                ROS_INFO("SUCCESS");
                ros::shutdown();
              }
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






//--------------------------------------------------------------
//old code


//    //calls BlobDetector to evaluate area
//     if (BlobDetector(mag_img))
//     {
//       if (!detection_status_)
//       {
//         //increment when endline not confirmed
//         hysteresis_counter_++;
//         if (hysteresis_counter_ > hysteresis_constant_)
//         {
//           //counter has passed threshold constant
//           hysteresis_counter_ = 0;
//           detection_status_ = true;
//           ROS_INFO("DETECTED");
//         }
//       }
//       else
//       {
//         //decay if detection not true
//         if (hysteresis_counter_)
//         {
//           hysteresis_counter_--;
//         }
//       }
//     }
//     else
//     {
//       //BlobCounter did not detect anything
//       if (!detection_status_)
//       {
//         //decay
//         if (hysteresis_counter_)
//         {
//           hysteresis_counter_--;
//         }
//       }
//       else
//       {
//         //post detection, detect when endline no longer in sight
//         hysteresis_counter_++;
//         if (hysteresis_counter_ > hysteresis_constant_)
//         {
//           hysteresis_counter_ = 0;
//           detection_status_ = false;
//           ROS_INFO("NO LONGER DETECTED");

//           //make service call
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
//   catch (cv_bridge::Exception& e)
//   {
//     ROS_ERROR("cv_bridge exception: %s", e.what());
//     return;
//   }
// }