Computer Vision Node
=========

This node handles all the computer vision tasks required for the [IAARC 2019](https://iarrc.org/) competition, which involves traffic light, traffic sign, lane, and endline detection.

### Lane Detection
All the races have lane lines present, and need to be reliably detected with varying lane widths and lighting conditions. For the 2019 competition, the races have the following lane widths:

* Drag Race: 1.5m
* Circuit Race: 2m
* Obstacle Avoidance Challenge: 2m
* Urban Road Challenge: 1m

The lane detection algorithm is as follows:

1. Adaptive threshold on the value (V) channel in the HSV colour space.
2. Warp transform to get a top-down view of the lanes.
3. Contour filtering and filling to get rid of noise (rocks reflecting light on the ground, shadow outlines, etc.) as well as fill in any gaps that appear in the lanes.
4. Convert filtered image (binary) to an occupancy grid, which is the data type that the mapper uses. Each race has calibrated values fed into the meta data for the mapper to ensure the lane width on the occupancy grid matches real life. See the bottom left corner of the occupancy grid images to see the lane width on the occupancy grid.

Here are the results of this algorithm on the 3 varying lane widths. From top to bottom, it's: Urban road (1m), Drag race (1.5m), and Circuit race (2m). Notice that some contours can make it through the contour filter if it's something significant like an outline of a major shadow or a very bright object, however it's always clear inside the lanes, which is what really matters.

| Original Image | 1. Adaptive Threshold | 2. Warp Transform | 3. Contour Filtering | 4. Occupancy Grid |
| -------------- | --------------------- | ----------------- | -------------------- | ----------------- |
| ![](images/lane_detection/urban_road/original.jpg) | ![](images/lane_detection/urban_road/threshold.jpg) | ![](images/lane_detection/urban_road/warp.jpg) | ![](images/lane_detection/urban_road/filtered.jpg) | ![](images/lane_detection/urban_road/occupancy.png) |
| ![](images/lane_detection/drag_race/original.jpg) | ![](images/lane_detection/drag_race/threshold.jpg) | ![](images/lane_detection/drag_race/warp.jpg) | ![](images/lane_detection/drag_race/filtered.jpg) | ![](images/lane_detection/drag_race/occupancy.png) |
| ![](images/lane_detection/circuit_race/original.jpg) | ![](images/lane_detection/circuit_race/threshold.jpg) | ![](images/lane_detection/circuit_race/warp.jpg) | ![](images/lane_detection/circuit_race/filtered.jpg) | ![](images/lane_detection/circuit_race/occupancy.png) |

### Traffic Light Detection
The start of each race is indicated by a traffic light switching from red to green. Our algorithm for detecting the traffic light is as follows:

1. Colour thresholding in the HSV colourspace to extract reds from the image. Read more about it [here](https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html). The thresholding values were specifically chosen based on testing on different rosbags taken in the E7 parking lot during the day time. It is important to notice that the results could vary under different lighting conditions. Below are two sets of images showing the result when facing towards and away from the sun:

| Original Image                                    |  Red Colour Threshold                          |
| ------------------------------------------------- | ---------------------------------------------- |
| ![](images/traffic_light_detection/original1.jpg) | ![](images/traffic_light_detection/thres1.png) |
| ![](images/traffic_light_detection/original2.jpg) | ![](images/traffic_light_detection/thres2.png) |

2. Find blobs in each frame by using OpenCV's blob detection algorithm. Read more about it [here](https://www.learnopencv.com/blob-detection-using-opencv-python-c/). The parameters in the algorithm are adjusted to catch blobs that look circular.

3. Sort and find the largest blob. Most of the time, the red light is the only blob that is detected and it is the largest blob. Therefore, it is safe to assume that the largest blob is coming from the traffic light.

4. Fit a bounding box around the blob found in the previous step, calculate the non-zero pixel ratio in the square, and set that as a reference ratio. 

5. Calculate the ratio on the same square for the next few frames and compare it to the reference value. Must see at least 10 frames in a row with a similar ratio before concluding that the red light has been detected. The 10 frame counter is to avoid false positives that could arise from noisy frames.

6. Keep checking the ratio in each frame until there are 5 frames in a row that give a ratio significantly smaller than the reference. This will happen when the red light turns off, at which point the green light comes on, and it is concluded that the green light has been detected.

7.	Once the green light is detected, a service call is made that is provided by the Supervisor node, which will figure out what to do next. The traffic light node shuts down after this service call is made.

### Endline Detection
All the races indicate the ending of a lap with a distinct magenta line, which is what we refer to as the "endline". Our algorithm for detecting the endline is as follows:

1. Apply colour thresholding with the bounds set to extract magenta in the HSV colour space. Read more about it [here](https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html). The thresholding values were realized through testing on endline tape setup in various lighting conditions. Note that lighting conditions could very likely give different results that don't look as good. Below are some images showing the result of this step:

| Original Image                              |  Magenta Colour Threshold                |
| ------------------------------------------- | ---------------------------------------- |
| ![](images/endline_detection/original1.jpg) | ![](images/endline_detection/thres1.jpg) |
| ![](images/endline_detection/original2.jpg) | ![](images/endline_detection/thres2.jpg) |

2. Extract contours from the thresholded image. Read more about it [here](https://docs.opencv.org/3.4/d4/d73/tutorial_py_contours_begin.html).

3. Sort contours by area and focus on the one with the largest area. We are assuming the contour with the largest area must be the endline. This is a pretty safe assumption in our case, the only way this backfires is if there is some large magenta object bigger than the endline in the robot's line of sight, which is unlikely.

4. For endline to be considered "detected", we must see 10 frames in a row with a max contour area above 1500. This number was picked by printing the contour areas to the terminal, and around the time the endline first comes into view, the area is around 1500. The 10 frame counter is to ensure no noisy frames trigger false positives.

5. Once endline is deteced, we now wait to see 10 frames in a row with a max contour area below 1500. Once this happens, a service call is made that is provided by the supervisor node, whose responsibility it is to then figure out what to do once the endline is gone. 
