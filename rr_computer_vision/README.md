Computer Vision Node
=========

This node handles all the computer vision tasks required for the [IAARC 2019](https://iarrc.org/) competition, which involves traffic light, traffic sign, lane, and endline detection.

### Endline Detection
All the races indicate the ending of a lap with a distinct magenta line, which is what we refer to as the "endline". Our algorithm for detecting the endline is as follows: 

1. Apply colour thresholding with the bounds set to extract magenta in the HSV colour space. Read more about it [here](https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html). The thresholding values were realized through testing the thresholding on a drag race setup in the E5 bay at night. Note that lighting conditions could very likely give different results that don't look as good. Below are some images showing the result of this step:

Original Image            |  Magenta Colour Threshold
:------------------------:|:-------------------------:
![](images/endline1.jpg)  |  ![](images/endline1_thres.jpg)
![](images/endline2.jpg)  |  ![](images/endline2_thres.jpg)

2. Extract contours from the thresholded image. Read more about it [here](https://docs.opencv.org/3.4/d4/d73/tutorial_py_contours_begin.html).

3. Sort contours by area and focus on the one with the largest area. We are assuming the contour with the largest area must be the endline. This is a pretty safe assumption in our case, the only way this backfires is if there is some large magenta object bigger than the endline in the robot's line of sight, which is unlikely.

4. For endline to be considered "detected", we must see 10 frames in a row with a max contour area above 1500. This number was picked by printing the contour areas to the terminal, and around the time the endline first comes into view, the area is around 1500. The 10 frame counter is to ensure no noisy frames trigger false positives.

5. Once endline is deteced, we now wait to see 10 frames in a row with a max contour area below 1500. Once this happens, a service call is made that is provided by the supervisor node, whose responsibility it is to then figure out what to do once the endline is gone. 

### Lane Detection
Based on different types of race, different configurations are defined in three .yaml files that will be loaded at the beginning.

First, a ros message is sent from left camera for drag race and urban challenge, right camera for circuit race. Tansforming it to cv::Mat with the following codes:

//cv::Mat img_input_bgr8;
//cv_bridge::CvImagePtr cv_bridge_bgr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//img_input_bgr8 = cv_bridge_bgr->image;

Then the image is converted to HSV and an adaptive thresholding on value channel is performed:

//cv::cvtColor(input_img, input_img, CV_BGR2HSV, 3);
//int patch_size_ = 25;
//std::vector<cv::Mat> channels;
//cv::split(input_img, channels);
//cv::adaptiveThreshold(channels[2], output_img, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY,patch_size_, -20);

The thresholded image is then passed into perpsective transform function to return a top-view image. Src coordiantes are defined in .yaml files depending on different types of race:

//cv::Mat dst = (cv::Mat_<float>(4,2) << 300.0, 0, 900.0, 0, 900.0, 710.0, 300.0, 710.0);
//cv::Mat transform_matrix = cv::getPerspectiveTransform(src, dst);
//cv::warpPerspective(input_img, output_img, transform_matrix, input_img.size());

The warped image is paased into findContours function that filters out small blobs and leaves only lane lines:

//cv::Mat filtered(img.size(), CV_8UC1, cv::Scalar(0));
//cv::Mat copy(img.clone());
  
//std::vector<std::vector<cv::Point> > contours;
//cv::findContours(copy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cv::Point());

//cv::Scalar color(255);
//for (size_t i = 0; i < contours.size(); i++) {
//  if (cv::contourArea(contours[i]) >= min_contour_size) {
//    cv::drawContours(filtered, contours, i, color, -1, 8, cv::noArray(), 2, cv::Point());
//  }
//}
//return filtered;

 
The output will pass through another bridge to be converted to ros messages with encoding type MONO8 because it's a binary image.

//cv_bridge::CvImage img_bridge_output;
//std_msgs::Header header; // empty header
//header.stamp = ros::Time::now(); // time
//img_bridge_output = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, out;
