import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
import os

MIN_MATCH_COUNT = 4

# Load in templates
templates = []
for subdir, dirs, files in os.walk("test_images/templates/"):
    for file in files:
        filepath = subdir + os.sep + file
        img = cv.imread(filepath, cv.IMREAD_GRAYSCALE)
        templates.append(img)

# Load in test image
load_test_img = cv.imread("test_images/image/haaltert.png", cv.IMREAD_GRAYSCALE)

# Crop image and increase constrast
test_img = cv.equalizeHist(load_test_img[400:600, 0:200])

# Increase contrast


cv.imshow('image',test_img)
cv.waitKey(0)
cv.destroyAllWindows()

# Initialize SIFT detector (try surf)
sift = cv.xfeatures2d.SIFT_create()

### Try feature matching with D1a_2 (8) and D5_2 (13) and B1 (3)
TEMPLATE_INDEX = 8
# Find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(test_img, None)
kp2, des2 = sift.detectAndCompute(templates[TEMPLATE_INDEX], None)

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)

flann = cv.FlannBasedMatcher(index_params, search_params)
matches = flann.knnMatch(des1, des2, k=2)

# Store all the good matches as per Lowe's ratio test
good = []
for m,n in matches:
    if m.distance < 0.7 * n.distance:
        good.append(m)

if len(good)>MIN_MATCH_COUNT:
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
    M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
    matchesMask = mask.ravel().tolist()
    h,w = test_img.shape
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv.perspectiveTransform(pts,M)
    img2 = cv.polylines(templates[TEMPLATE_INDEX],[np.int32(dst)],True,255,3, cv.LINE_AA)
else:
    print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
    matchesMask = None

draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)
img3 = cv.drawMatches(test_img,kp1,img2,kp2,good,None,**draw_params)
plt.imshow(img3, 'gray'),plt.show()

