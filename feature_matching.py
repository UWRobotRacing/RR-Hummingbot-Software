import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

# Load in templates
template_names = ("left.jpeg", "right.jpeg", "straight.jpeg")
templates = []

for name in template_names:
    img = cv.imread("templates/" + name, cv.IMREAD_GRAYSCALE)
    templates.append(img)

# Load in test image
test_img = cv.imread("test_images/image/haaltert.png", cv.IMREAD_GRAYSCALE)

# cv2.imshow('image',templates[0])
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# Initialize SIFT detector (try surf)
sift = cv.xfeatures2d.SIFT_create()

# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(test_img,None)
kp2, des2 = sift.detectAndCompute(img2,None)

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)

flann = cv.FlannBasedMatcher(index_params, search_params)
matches = flann.knnMatch(des1,des2,k=2)




