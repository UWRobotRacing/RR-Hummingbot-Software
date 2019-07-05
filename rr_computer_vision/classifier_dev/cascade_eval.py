import numpy as np 
import cv2
from timeit import default_timer as timer
import os

# File used to inspect Haar cascade output on images and look at average evaluation time

images_dir = 'C:/Users/ethie/OneDrive - University of Waterloo/Documents/UWRobotics/StreetImageDataset/synthetic_data/haar_training_data/positives/images'
classifier = cv2.CascadeClassifier('C:/Users/ethie/OneDrive - University of Waterloo/Documents/UWRobotics/classifier_v2/cascade_stage13.xml')

total_time = 0
count = 0
# Loop through all images in the directory
for f in os.listdir(images_dir):
    # Load in image and convert to grayscale
    img = cv2.imread(os.path.join(images_dir, f))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Time how long it takes to evaluate image
    start = timer()
    signs = classifier.detectMultiScale(gray, 1.2, 2)
    end = timer()

    total_time += end - start
    count += 1

    print(count)
    print(end - start)

    for (x,y,w,h) in signs:
        # Overlay bounding boxes over image for visualization
        img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]
    
    # cv2.imshow('image',img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

# Get average evaluation time 
print(total_time/count)