import numpy as np
import cv2
from matplotlib import pyplot as plt

# Global parameters
EPSILON = 0.01
TIP_ANGLE = 79

# Load in template
img = cv2.imread("templates/SIGN_UP.jpg", cv2.IMREAD_GRAYSCALE)
rows,cols = img.shape

# Rotate image by 90 deg
M = cv2.getRotationMatrix2D((cols/2,rows/2), 90, 1)
rotated = cv2.warpAffine(img, M, (cols,rows))

# Convert to binary
ret, thresh = cv2.threshold(rotated, 100, 255, 0)

# Find contours and grab largest contour
im2, cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
sorted_cnts = sorted(cnts, key=lambda x: cv2.contourArea(x))
cnt = sorted_cnts[-1]

# Fit polygon to contour
perimeter = cv2.arcLength(cnt,True)
approx = cv2.approxPolyDP(cnt,EPSILON*perimeter,True)

# Organize polygon into x and y vertex coordinates
x = []
y = []
for pnt in approx:
    x.append(pnt[0][0])
    y.append(pnt[0][1])

plt.imshow(im2, cmap="gray")
#colors = ['r','g','b','c','m','y','k']

angles = []
directions = []
for i in range(len(x)):
    #plt.plot([x[i], x[(i+1)%7]],[y[i], y[(i+1)%7]], colors[i])

    # Convert triplets of points to vectors
    vect1 = [x[i]-x[(i+1)%7], y[i]-y[(i+1)%7]]
    vect2 = [x[(i+2)%7]-x[(i+1)%7], y[(i+2)%7]-y[(i+1)%7]]

    # Find angle between points using dot product
    dot_prod = vect1[0]*vect2[0] + vect1[1]*vect2[1]
    mag1 = (vect1[0]**2 + vect1[1]**2)**0.5
    mag2 = (vect2[0]**2 + vect2[1]**2)**0.5 

    angle = np.arccos(dot_prod/(mag1*mag2))
    angles.append(angle)

    # Find direction of corner by taking negative of the vector that bisects both vectors
    bisector = [vect1[0]/mag1+vect2[0]/mag2, vect1[1]/mag1+vect2[1]/mag2]
    direction = np.arctan2(-bisector[1], -bisector[0])

    directions.append(direction)

plt.show()

# Convert to degrees
angles = [i*180/np.pi for i in angles]
directions = [i*180/np.pi for i in directions]

# Angle of the tip of the arrow is always ~79.6 deg,
# other angles are ~90 and ~50 deg
# Find tip using angle nearest to 79 degrees
tip_angle_index = min(range(len(angles)), key=lambda i: abs(angles[i]-TIP_ANGLE))
arrow_direction = directions[tip_angle_index]

if -135 <= arrow_direction < -45:
    print('up')
elif -45 <= arrow_direction <= 45:
    print('right')
elif arrow_direction < -135 or arrow_direction >= 135:
    print('left')
else:
    print('no result')
