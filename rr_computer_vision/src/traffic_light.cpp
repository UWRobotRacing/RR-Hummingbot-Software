/** @file trsffic light detection
 *  @author Yuchi(Allan) Zhao
 *  @competition IARRC 2019
 */

#include "traffic_light.hpp"

TrafficLightProcessor::TrafficLightProcessor(ros::NodeHandle nh) : it_(nh_)  {

    red_light_detected = false;
    
    //cv::Rect boundRect;
    // cv::Mat rectSection;
    red_Pixel_Counter=0;
    default_ratio=0.0;
    
    test_subscriber = it_.subscribe("/zed/rgb/image_rect_color", 1, &TrafficLightProcessor::TrafficLightImageCallback, this);
    test_publisher = it_.advertise("/test_traffic_light", 1);

    client_ = nh_.serviceClient<std_srvs::Empty>("/Supervisor/start_race");
}

void TrafficLightProcessor::TrafficLightImageCallback(const sensor_msgs::ImageConstPtr& msg) {

  cv::Mat traffic_light_image, hsv_img, mag_img, blur_img;
  cv_bridge::CvImagePtr cv_ptr;
  std_srvs::Trigger srv;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<cv::Point> approx;
  double new_ratio=0.0;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    traffic_light_image = cv_ptr->image;
    cv::cvtColor(traffic_light_image, hsv_img, CV_BGR2HSV);
   
    // Filter red
      //NEED CHANGE
    cv::inRange(hsv_img, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mag_img);
    cv::Mat red_light_img;
    cv::GaussianBlur(red_light_img, red_light_img, cv::Size(7,7), 0, 0);

    // PUBLISH and visualize in rviz   
    cv_bridge::CvImage img_bridge_output;
    std_msgs::Header header;
    header.stamp=ros::Time::now();
    img_bridge_output=cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, red_light_img);
    test_publisher.publish(img_bridge_output.toImageMsg());

    // // find contour and detect circle
    if(red_light_detected==false){
        cv::findContours(mag_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
        int conSize = contours.size();
        ROS_INFO("Number of contours: %d", conSize);

        int maxAreaIndex=0;
        for (int i=0; i<conSize; i++){
          if(cv::contourArea(contours[maxAreaIndex]) < cv::contourArea(contours[i+1])){
            maxAreaIndex=i+1;
          }
        }

        cv::approxPolyDP(cv::Mat(contours[maxAreaIndex]), approx, 0.1*arcLength(cv::Mat(contours[maxAreaIndex]), true), true);
        int polyLength = approx.size();
        ROS_INFO("Polygon length: %d", polyLength);
        
        if (approx.size()>=10){
            boundRect = cv::boundingRect(contours[maxAreaIndex]);    // cv::Mat(contours[maxAreaIndex])
            cv::Mat crop_img=mag_img(boundRect);
            cv::cvtColor(crop_img, crop_img, CV_BGR2GRAY);
            int red_Pixel_Counter=cv::countNonZero(crop_img);
          
            int total_pixel=crop_img.total();
            default_ratio=(double)red_Pixel_Counter/total_pixel;
            ROS_INFO("Red light detected!");
            red_light_detected=true;
        } 
    }else if(red_light_detected=true){
        cv::Mat crop_img=mag_img(boundRect);
        cv::cvtColor(crop_img, crop_img, CV_BGR2GRAY);
        int red_Pixel_Counter=cv::countNonZero(crop_img);
        int total_pixel=crop_img.total();
        new_ratio= (double)red_Pixel_Counter/total_pixel;
        if(new_ratio>default_ratio){
            ROS_INFO("Red light is there!");
        }else{
            ROS_INFO("Green light detected!");
            ros::shutdown();
        }
    }

  }catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}



// NO SORTING
// for (int i=0; i<conSize; i++){
//     cv::approxPolyDP(cv::Mat(contours[i]), approx, 0.1*arcLength(cv::Mat(contours[i]), true), true);
//     int polyLength = approx.size();
//     ROS_INFO("Polygon length: %d", polyLength);

//     // determine if it is circle
//       //buffer (it also helps eliminate noise )
//     if (approx.size()>=15){
//       red_light_counter++;
//     } else if(red_light_counter >= 5){
//       is_red_light=true;
//       red_light_counter=0;
//     }
//   }