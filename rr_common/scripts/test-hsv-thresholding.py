# Used code from the following tutorial:
# https://docs.opencv.org/3.4.3/da/d97/tutorial_threshold_inRange.html

import cv2
import argparse
import sys
from pathlib import Path

max_value = 255
max_value_H = 360//2
low_H = 130
low_S = 62
low_V = 55
high_H = 180
high_S = 255
high_V = 255
window_capture_name = 'Pre Threshold'
window_detection_name = 'Detection image'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv2.setTrackbarPos(low_H_name, window_detection_name, low_H)
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv2.setTrackbarPos(high_H_name, window_detection_name, high_H)
def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv2.setTrackbarPos(low_S_name, window_detection_name, low_S)
def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv2.setTrackbarPos(high_S_name, window_detection_name, high_S)
def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv2.setTrackbarPos(low_V_name, window_detection_name, low_V)
def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv2.setTrackbarPos(high_V_name, window_detection_name, high_V)

def main(image_path):
    img = cv2.imread(image_path)

    cv2.namedWindow(window_capture_name, cv2.WINDOW_NORMAL);
    cv2.namedWindow(window_detection_name, cv2.WINDOW_NORMAL);

    cv2.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
    cv2.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
    cv2.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
    cv2.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
    cv2.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
    cv2.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)

    while True:
        img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_threshold = cv2.inRange(img_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))

        cv2.imshow(window_capture_name, img)
        cv2.imshow(window_detection_name, img_threshold)

        key = cv2.waitKey(30)
        if key == ord('q') or key == 27:
            break

if __name__ == "__main__":
    # Get command line arguments
    PARSER = argparse.ArgumentParser()
    PARSER.add_argument('image_path', type = str, default = None,
                        help = 'path to image from current directory')
    ARGS = PARSER.parse_args()
    image_path = Path(ARGS.image_path)

    # Check to make sure image path exists
    if not image_path.exists():
        print("Image path %s does not exist!" % image_path)
        sys.exit(1)

    main(str(image_path))
