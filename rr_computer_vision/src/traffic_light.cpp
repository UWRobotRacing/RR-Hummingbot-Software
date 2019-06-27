/** @file trsffic light detection
 *  @author Yuchi(Allan) Zhao
 *  @competition IARRC 2019
 */

#include "traffic_light.hpp"

TrafficLightProcessor::TrafficLightProcessor(ros::NodeHandle nh) : it_(nh_)  {

    red_light_detected = false;
    cv::Rect boundRect;
    red_Pixel_Counter=0;
    default_ratio=0.0;
    int x;
    int y;
    int s;
    int r;
    
    test_subscriber = it_.subscribe("/zed/left/image_rect_color", 1, &TrafficLightProcessor::TrafficLightImageCallback, this);
    test_publisher = it_.advertise("/test_traffic_light", 1);

    client_ = nh_.serviceClient<std_srvs::Empty>("/Supervisor/start_race");
}

void TrafficLightProcessor::TrafficLightImageCallback(const sensor_msgs::ImageConstPtr& msg) {

  cv::Mat traffic_light_image, hsv_img, threshold_img, blur_img;
  cv_bridge::CvImagePtr cv_ptr;
  std_srvs::Trigger srv;
  double new_ratio;
//  std::vector<cv::Point> approx;

  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    traffic_light_image = cv_ptr->image;
    cv::cvtColor(traffic_light_image, hsv_img, CV_BGR2HSV);
    int maxAreaIndex = 0;
    int counter=0;

    // Filter red
    cv::inRange(hsv_img,cv::Scalar(0, 62, 161), cv::Scalar(96, 255, 215), threshold_img);    //outdoor :cv::Scalar(0, 62, 161), cv::Scalar(96, 255, 215)    cv::Scalar(0, 44, 40), cv::Scalar(96, 189, 160)
    cv::GaussianBlur(threshold_img, threshold_img, cv::Size(7,7), 0, 0);

    if(red_light_detected==false){
        cv::SimpleBlobDetector::Params params;
        params.filterByArea = true;
        params.minArea = 50;
        
        // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = 0.3;
        
        // Filter by Convexity
        params.filterByConvexity = true;
        params.minConvexity = 0.4;
        
        // Filter by Inertia
        params.filterByInertia = false;
        //params.minInertiaRatio = 0.01;

        params.blobColor=255;
        
        // Set up detector with params
        // OR 
        // Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        cv::SimpleBlobDetector detector(params);    
        
        // Detect blobs.
        std::vector<KeyPoint> keypoints;
        detector.detect( threshold_img, keypoints);

        int numBlob=keypoints.size();
        ROS_INFO("Number of blobs: %i", numBlob);

        if(numBlob !=0 ){
            for (int i = 0; i < numBlob; i++)
            {
              if ( keypoints[maxAreaIndex].size) < keypoints[i].size)
              {
                maxAreaIndex = i;
              }
            }

            x=(int)keypoints[0].pt[0];
            y=(int)keypoints[0].pt[1];
            s=(int)keypoints[maxAreaIndex].size;
            r=(int) s/2;

            cv::Mat im_with_keypoints;
            cv::drawKeypoints( threshold_img, keypoints, im_with_keypoints, Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

            // PUBLISH and visualize in rviz   
            cv_bridge::CvImage img_bridge_output;
            std_msgs::Header header;
            header.stamp=ros::Time::now();
            img_bridge_output=cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, im_with_keypoints);
            test_publisher.publish(img_bridge_output.toImageMsg());
     
            cv::Rect boundRect (x-r, y-r, x+s, y+s);   
            cv::Mat crop_img=threshold_img(boundRect);
            int red_Pixel_Counter=cv::countNonZero(crop_img);
            int total_pixel=crop_img.total();
            
            if(counter==0){
                default_ratio=(double)red_Pixel_Counter/total_pixel;
                counter++;
            } else {
              new_ratio=(double)red_Pixel_Counter/total_pixel;              
              if((new_ratio+0.2)>= default_ratio){
                  counter++;
                  ROS_INFO("Counter: %i", counter);
              }
            }
            
            if(counter==5){
                ROS_INFO("Red light detected!");
                ROS_INFO("Default ratio: %f", default_ratio);
                red_light_detected=true;
            }
        }
        
    }else if(red_light_detected==true){
        cv::Mat crop_img=threshold_img(boundRect);
        int red_Pixel_Counter=cv::countNonZero(crop_img);
        int total_pixel=crop_img.total();
        new_ratio= (double)red_Pixel_Counter/total_pixel;
        ROS_INFO("RATIO: %f", new_ratio);
        if((new_ratio+0.3)>= default_ratio){
            ROS_INFO("Red light is still there!");
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
