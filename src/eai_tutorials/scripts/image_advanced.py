import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('../images/flower.jpg')
cv2.imshow('Original',img)

# Translation
H = np.float32([[1,0,100],[0,1,50]])
rows,cols = img.shape[:2]
res = cv2.warpAffine(img,H,(rows,cols))
cv2.imshow('Translation',res)

cv2.waitKey()
cv2.destroyWindow('Translation')

# Scaling
res1 = cv2.resize(img,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC)
height,width = img.shape[:2]
res2 = cv2.resize(img,(2*width,2*height),interpolation=cv2.INTER_CUBIC)
cv2.imshow('Scaling',res2)

cv2.waitKey()
cv2.destroyWindow('Scaling')

# Rotation
rows,cols = img.shape[:2]
M = cv2.getRotationMatrix2D((cols/2,rows/2),45,1)
res = cv2.warpAffine(img,M,(rows,cols))
cv2.imshow('Rotation',res)

cv2.waitKey()
cv2.destroyWindow('Rotation')

# Affine --fang3_she4
rows,cols = img.shape[:2]
pts1 = np.float32([[50,50],[200,50],[50,200]])
pts2 = np.float32([[10,100],[200,50],[100,250]])
M = cv2.getAffineTransform(pts1,pts2)
res = cv2.warpAffine(img,M,(rows,cols))
cv2.imshow('Affine: Rotation + stretching )',res)

cv2.waitKey()
cv2.destroyWindow('Affine: Rotation + stretching )')

# Transmittance
rows,cols = img.shape[:2]
pts1 = np.float32([[56,65],[238,52],[28,237],[239,240]])
pts2 = np.float32([[0,0],[200,0],[0,200],[200,200]])
M = cv2.getPerspectiveTransform(pts1,pts2)
res = cv2.warpPerspective(img,M,(200,200))
cv2.imshow('Transmittance',res)

cv2.waitKey()
cv2.destroyAllWindows()