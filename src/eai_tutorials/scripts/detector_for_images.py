#!/usr/bin/env python
import cv2
filename = '../images/faces.jpg'

face_cascade = cv2.CascadeClassifier('./cascades/haarcascade_frontalface_default.xml')
face_cascade.load('./cascades/haarcascade_frontalface_default.xml')

img = cv2.imread(filename)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
faces = face_cascade.detectMultiScale(gray, 1.3 , 5)
for (x,y,w,h) in faces:
    img = cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)
cv2.namedWindow('faces detected')
cv2.imshow('faces detected', img)
cv2.waitKey(0)
