/** @file thresholding_values.hpp
 *  @author Waleed Ahmed (w29ahmed)
 *  @breif Thresholding values used throughout the cv node
 *  @competition IARRC 2019
 */
#ifndef THRESHOlDING_VALUES
#define THRESHOlDING_VALUES

#include <opencv2/opencv.hpp>

enum weather_conditions {
  overcast = 0,
  sunny = 1,
  sun_in_image = 2,
  indoor = 3
};

namespace traffic_light {
  // The traffic light looks more white when indoors and overcast as it is very bright LEDs 
  // that can easily oversaturate image sensors
  const cv::Scalar overcast_hsv_lower_bounds(0, 0, 230);
  const cv::Scalar overcast_hsv_upper_bounds(180, 40, 255);

  const cv::Scalar indoor_hsv_lower_bounds(0, 0, 230);
  const cv::Scalar indoor_hsv_upper_bounds(180, 40, 255);

  // When sunny, the traffic light looks like more like a consistent red
  const cv::Scalar sunny_hsv_lower_bounds(0, 90, 155);
  const cv::Scalar sunny_hsv_upper_bounds(15, 255, 255);

  const cv::Scalar sun_in_image_hsv_lower_bounds(0, 90, 155);
  const cv::Scalar sun_in_image_hsv_upper_bounds(15, 255, 255);
}

namespace endline_detection {
  // Overcast and sunny are the same since endline is a very distinct colour and 
  // is caught easily by hsv thresholding
  const cv::Scalar overcast_hsv_lower_bounds(130, 62, 55);
  const cv::Scalar overcast_hsv_upper_bounds(180, 255, 255);

  const cv::Scalar sunny_hsv_lower_bounds(130, 62, 55);
  const cv::Scalar sunny_hsv_upper_bounds(180, 255, 255);

  // Distinct case for sun being in image as it oversaturates the image sensor and 
  // white balancing from the camera makes the rest of the scene very dark
  const cv::Scalar sun_in_image_hsv_lower_bounds(130, 0, 41);
  const cv::Scalar sun_in_image_hsv_upper_bounds(180, 255, 177);

  const cv::Scalar indoor_hsv_lower_bounds(130, 82, 158);
  const cv::Scalar indoor_hsv_upper_bounds(180, 255, 255);
}

#endif //THRESHOlDING_VALUES