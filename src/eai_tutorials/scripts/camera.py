#!/usr/bin/env python
import cv2
 
camera = cv2.VideoCapture(0)
while True:
    ret, frame = camera.read()
    cv2.imshow('Camera',frame)
    if cv2.waitKey(20) & 0xff == 27: #esc
        break

cv2.destroyAllWindows()
cameraCapture.release()