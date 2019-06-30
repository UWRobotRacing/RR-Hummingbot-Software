  //  @file traffic light detection
  //  @author Yuchi(Allan) Zhao
  //  @competition IARRC 2019

#include "traffic_light.hpp"

TrafficLightProcessor::TrafficLightProcessor(ros::NodeHandle nh) : it_(nh_)  {
    red_light_detected = false;
    cv::Rect boundRect;
    red_Pixel_Counter=0;
    default_ratio=0.0;
    x=0;
    y=0;
    s=0;
    r=0;
    redLightCounter=0;
    greenLightCounter=0;
    // test_subscriber = it_.subscribe("/zed/left/image_rect_color", 1, &TrafficLightProcessor::TrafficLightImageCallback, this);
    // test_publisher = it_.advertise("/test_traffic_light", 1);
    client_ = nh_.serviceClient<std_srvs::Empty>("/Supervisor/start_race");
}

void TrafficLightProcessor::TrafficLightImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat traffic_light_image, hsv_img, threshold_img, blur_img, im_with_keypoints;
  cv_bridge::CvImagePtr cv_ptr;
  std_srvs::Trigger srv;
  double new_ratio=0;

  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    traffic_light_image = cv_ptr->image;
    cv::cvtColor(traffic_light_image, hsv_img, CV_BGR2HSV);
    int maxAreaIndex = 0;

    // Filter red
    cv::inRange(hsv_img,cv::Scalar(0, 90, 155), cv::Scalar(15, 255, 255), threshold_img);
    cv::GaussianBlur(threshold_img, threshold_img, cv::Size(7,7), 0, 0);

    // PUBLISH and visualize in rviz   
    // cv_bridge::CvImage img_bridge_output;
    // std_msgs::Header header;
    // header.stamp=ros::Time::now();
    // img_bridge_output=cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, threshold_img);
    // test_publisher.publish(img_bridge_output.toImageMsg());

    if(red_light_detected==false){
        // set parameter for blob detection
        cv::SimpleBlobDetector::Params params;
        
        // Filter by Area
        params.filterByArea = true;
        params.minArea = 300;
        
        // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = 0.6;
        
        // Filter by Convexity
        params.filterByConvexity = true;
        params.minConvexity = 0.6;
        
        // Filter by Inertia
        params.filterByInertia = false;
        //params.minInertiaRatio = 0.01;

        // Filter white
        params.blobColor=255;
        
        // Set up detector with params
        cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);    
        
        // Detect blobs.
        std::vector<cv::KeyPoint> keypoints;
        detector->detect( threshold_img, keypoints);
        int numBlob=keypoints.size();
        // ROS_INFO("Number of blobs: %i", numBlob);

        if(numBlob > 0){
            // find the largest blob if there are more than 2 blobs
            if(numBlob > 1){
              for (int i = 0; i < numBlob; i++)
              {
                if (keypoints[maxAreaIndex].size < keypoints[i+1].size)
                {
                  maxAreaIndex = i+1;
                }
              }
            }

            // the center and diameter of the blob
            x=(int)keypoints[maxAreaIndex].pt.x;
            y=(int)keypoints[maxAreaIndex].pt.y;
            s=(int)keypoints[maxAreaIndex].size;
            r=(int) s/2;
            // ROS_INFO("X: %i", x);
            // ROS_INFO("Y: %i", y);
            // ROS_INFO("S: %i", s);
            // ROS_INFO("R: %i", r);
            // cv::rectangle(threshold_img, cv::Point(x-r, y-r), cv::Point(x+s, y+s), cv::Scalar(0, 0, 255), 2,8);
            
            // create a reatangle around the blob
            cv::Rect firstRect (x-r, y-r, s, s);   
            cv::Mat crop_img=threshold_img(firstRect);
            boundRect=firstRect;

            // count pixel in the rectangle
            int red_Pixel_Counter=cv::countNonZero(crop_img);
            int total_pixel=crop_img.total();
            
            if(redLightCounter == 0){
                default_ratio=(double)red_Pixel_Counter/total_pixel;
                // ROS_INFO("default ratio: %f", default_ratio);
                redLightCounter++;
            } else {
              new_ratio=(double)red_Pixel_Counter/total_pixel; 

              // red light buffer             
              if((new_ratio+0.1)>= default_ratio){
                  redLightCounter++;
              }
            }
            
            if(redLightCounter>=10){
                ROS_INFO("Red light detected!");
                red_light_detected=true;
                redLightCounter=0;
            }
        }
    }else {
        
        //crop the existing rectangle on each new frame and count pixel
        cv::Mat crop_img=threshold_img(boundRect);
        int red_Pixel_Counter=cv::countNonZero(crop_img);
        int total_pixel=crop_img.total();
        new_ratio= (double)red_Pixel_Counter/total_pixel;

        if((new_ratio+0.1)>= default_ratio){
            ROS_INFO("Red light is still there!");
        }else{
            // green light buffer
            greenLightCounter++;
            if(greenLightCounter >= 5){
              ROS_INFO("Green light detected!");
              if (client_.call(srv)){
                ros::shutdown();
              }
            }
        }
    }

  }catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}