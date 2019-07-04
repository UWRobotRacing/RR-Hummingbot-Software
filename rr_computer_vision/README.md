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

### Sign Detection
For the urban challenge, an important part of the race is the detection and classification of the signs that will tell the robot where to go through an intersection. Using only the right rectified camera feed from our ZED Stereo Camera, we were able to get a robust sign detection and classification algorithm.

Before implementing our algorithm in ROS, we had to find a way to detect the sign from the camera input. We decided to train our own Haar Cascade classifier to detect these signs. The reasons for choosing a Haar Cascade over other more state-of-the-art techniques such as R-CNNs and Single Shot Detectors are as follows:
1. Haar cascades can evaluate images significantly faster than other CNN based methods on both CPU and GPU
2. A state-of-the-art CNN model was found to be unnecessary because of the simplicity of the object being detected and distinct Haar like features that could be found from them.

A custom training dataset was created by taking an existing traffic sign dataset, found [here](https://www.mapillary.com/dataset/vistas?pKey=cc5dEAyQECBFF9MN3MbdZA), and overlaying the given sign template over top with some random transformations applied to the sign, creating approximately 4000 positive images and 14000 negative images. The Haar Cascade was then trained using [OpenCV](https://docs.opencv.org/3.3.0/dc/d88/tutorial_traincascade.html)'s implementation over the span of 2 days for 14 stages.

With the trained Haar Cascade, our algorithm to acquire the arrow direction from ZED Camera images is as follows:
1. Apply the cascade to the input image to get a collection of bounding boxes that the classifier thinks are signs. In order to only get one sign as output we grab the bounding box with the highest certainty as output by the cascade.
2. To ensure our bounding box is in fact the sign and not a false positive, we ensure that we have been tracking this specific bounding box for a minimum of 10 consecutive frames. With the correct bounding box, we crop out this section of the image and feed it to our arrow classification algorithm.
3. The arrow classification algorithm starts by applying Canny edge detection to the sub-image and then applies a morphological closing operation to close any gaps in the Canny edges.
4. We then find all of the contours in the edge detection image and take the contour with the closest Euclidean distance to the center of the image. This can be assumed to be the arrow since the bounding box is centered around the sign.
5. Next, an ellipse is fit to the arrow contour in order to get an estimate of the arrow's center and orientation. If the orientation is vertical, we can conclude that the arrow is indicating us to go straight. If the orientation is horizontal, we can assume that the arrow is either indicating us to go left or right, and some further processing must be performed. In order to differentiate left from right, we start by finding a line that passes through the found ellipse's center and runs perpendicular to it's major axis.
6. From this line, we create two masks that split the arrow in half. One mask contains the arrow's head and one contains the arrow's base.
7. Finally, we can mask the arrow using both masks and count the number of pixels in both sections of the arrow. It can be assumed that the section of the arrow containing the head contains more pixels than the other section. We now know the orientation of the arrow head and can conclude if we need to turn left or right.

Example Haar Cascade output:

Original image | Haar cascade bounding box
:-------------:|:------------------------:
![](images/sign_detection/zed_image.jpg) | ![](images/sign_detection/bounding_box.jpg)

Example images for arrow classification algorithm:

Canny edge detection | Arrow contour | Fit ellipse and line | Masks used to split arrow
:-------------------:|:-------------:|:--------------------:|:-------------------------:
![](images/sign_detection/canny_output.jpg) |  ![](images/sign_detection/arrow_binary.jpg) | ![](images/sign_detection/ellipse_fit.jpg) |  ![](images/sign_detection/split_masks.jpg)

