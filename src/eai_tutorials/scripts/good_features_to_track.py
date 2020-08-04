#!/usr/bin/env python
import numpy as np  
import cv2  

def getkpoints(imag,input1):   
    x,y=0,0 
    w1,h1=input1.shape   
    input1=input1[0:w1,200:h1]  
    try:
        w,h=imag.shape  
    except:
        return None  
  
    keypoints=list()  
    kp=cv2.goodFeaturesToTrack(input1,100,0.02,7)                                 
      
    if kp is not None and len(kp)>0:  
        for x,y in np.float32(kp).reshape(-1,2):  
            keypoints.append((x,y))  
    return keypoints  

  
def process(image):  
    grey1=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)  
    grey=cv2.equalizeHist(grey1)  
    keypoints=getkpoints(grey,grey1)  

    if keypoints is not None and len(keypoints)>0:  
        for x,y in keypoints:  
            cv2.circle(image, (int(x+200),y), 3, (255,255,0))  
    return image  


camera=cv2.VideoCapture(0)  
keypoints=list()

while camera.isOpened():  
    ret,frame=camera.read()  
    frame=process(frame)  
    cv2.imshow('frame',frame)  
    if cv2.waitKey(1)&0xFF==27:  
        break

cv2.destroyAllWindows()