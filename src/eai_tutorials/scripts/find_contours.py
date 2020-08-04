#!/usr/bin/env python
import cv2
import numpy as np

img = cv2.pyrDown(cv2.imread("../images/cross.jpg",cv2.IMREAD_UNCHANGED))
cv2.imshow("original",img)
ret, thresh = cv2.threshold(cv2.cvtColor(img.copy(),cv2.COLOR_BGR2GRAY),127,255,cv2.THRESH_BINARY)
image, contours, hier = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

for c in contours:
    x,y,w,h = cv2.boundingRect(c)
    cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)

    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.drawContours(img,[box],0,(0,255,0),2)

    (x,y),radius = cv2.minEnclosingCircle(c)
    center = (int(x),int(y))
    radius = int(radius)
    img = cv2.circle(img, center,radius, (0,0,255), 2)

    epsilon = 0.01 * cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, epsilon, True)
    hull = cv2.convexHull(c)

length_hull=len(hull)-2
if length_hull>2:
    for i in range(0,len(hull)-2):
        cv2.line(img,(hull[i][0][0],hull[i][0][1]),(hull[i+1][0][0],hull[i+1][0][1]),(255,255,0),2)
    cv2.line(img,(hull[0][0][0],hull[0][0][1]),(hull[i+1][0][0],hull[i+1][0][1]),(255,255,0),1)

cv2.imshow('Original',image)
cv2.imshow('Contours',img)
cv2.waitKey()
cv2.destroyAllWindows()
