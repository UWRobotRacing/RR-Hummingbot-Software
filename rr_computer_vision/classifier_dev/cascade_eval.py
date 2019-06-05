import numpy as np 
import cv2

classifier = cv2.CascadeClassifier('haarcascade_traffic_sign.xml')

img = cv2.imread('test_image.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

signs = face_cascade.detectMultiScale(gray, 1.3, 5)
for (x,y,w,h) in signs:
    img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
    roi_gray = gray[y:y+h, x:x+w]
    roi_color = img[y:y+h, x:x+w]

cv2.imshow('img',img)
cv2.waitKey(0)
cv2.destroyAllWindows()