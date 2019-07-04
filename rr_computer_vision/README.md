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

Original Image            |  Red Colour Threshold
:------------------------:|:-------------------------:
![](images/trafficLight1.jpg)  |  ![](images/trafficLight1_thre.jpg)
![](images/trafficLight2.jpg)  |  ![](images/trafficLight2_thre.jpg)
