import cv2
import argparse
import sys
from pathlib import Path

window_capture_name = "Pre Threshold"
window_detection_name = "Detection Image"

low_value = 0
high_value = 255
max_value = 255
low_name = 'Low Value'
high_name = 'High Value'

def on_low_thresh_trackbar(val):
    global low_value
    global high_value
    low_value = val
    low_value = min(high_value-1, low_value)
    cv2.setTrackbarPos(low_name, window_detection_name, low_value)
    
def on_high_thresh_trackbar(val):
    global low_value
    global high_value
    high_value = val
    high_value = max(high_value, low_value+1)
    cv2.setTrackbarPos(high_name, window_detection_name, high_value)

def main(image_path):
    img = cv2.imread(image_path)
    
    cv2.namedWindow(window_capture_name, cv2.WINDOW_NORMAL);
    cv2.namedWindow(window_detection_name, cv2.WINDOW_NORMAL);
    
    cv2.createTrackbar('Low Value', window_detection_name , low_value, max_value, on_low_thresh_trackbar)
    cv2.createTrackbar('High Value', window_detection_name , high_value, max_value, on_high_thresh_trackbar)
    
    while True:
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_threshold = cv2.inRange(img_gray, low_value, high_value)
        
        cv2.imshow(window_capture_name, img_gray)
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
