At the beginning, the program reveives a ros message from the subscriber, tansform it to cv::Mat, and transform it to HSV colorspace with the following code:

//cv_bridge::CvImagePtr cv_bridge_bgr = cv_bridge::toCvCop(msg,sensor_msgs::image_encodings::BGR8);
//cv::Mat img_bgr8 = cv_bridge_bgr->image;
//cv::cvtColor(img_bgr8, Im1_HSV_, CV_BGR2HSV, 3);

To perform perspective transform, world coordinates and image coordiantes are selected by trial and error. Src represents world coordiantes and dst represents image coordinates.

//src = (cv::Mat_<float>(4,2) << 200.0, 400.0, 1150.0, 400.0, 1280.0, 500.0, 20, 580.0);
//dst = (cv::Mat_<float>(4,2) << 300.0, 0, 900.0, 0, 900.0, 730.0, 300.0, 730.0);

Perspective transform matrix M is calculated and used to perform perspective transform. The function takes in the HSV image and returns a brids-eye view.

//M = cv::getPerspectiveTransform(src, dst);
//cv::warpPerspective(Im1_HSV_,BEV_image,M,img_gray.size());

The BEV image is passed into 2 functions: Multithreshold() and FindWhite(). Both of them try to find possible lane pixels and return 2 binary images. The binary images are combined using 'or' operation.

//multibounds_ = (cv::Mat_<double>(3,6) << 0, 100, 140, 120, 255, 255, 0, 0, 250, 255, 25, 255, 					  25, 5, 186, 130, 50, 255);
//Multithreshold(BEV_image, multibounds_, mask_warped_1_);

//bounds_ = cv::Scalar(20, -40);
//adapt_hsv_patch_size_ = 25;
//FindWhite(BEV_image, bounds_, adapt_hsv_patch_size_, mask_warped_2_);

//cv::bitwise_or(mask_warped_1_, mask_warped_2_, mask_warped_1_);

The combined image will be passed into GetContours() function, which will find contours that's above the defined minimum blob size, and outputs the final output image.

//blob_size_ = 100;
//out = GetContours(mask_warped_1_, blob_size_);

 
The output will pass through another bridge to be converted to ros messages with encoding type MONO8 because it's a binary image.

//cv_bridge::CvImage img_bridge_output;
//std_msgs::Header header; // empty header
//header.stamp = ros::Time::now(); // time
//img_bridge_output = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, out;
