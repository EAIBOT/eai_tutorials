#!/usr/bin/env python
import cv2
import numpy as np
from scipy import ndimage

img = cv2.imread("../images/flower.jpg",0)

kernel_3x3 = np.array([[-1,-1,-1],
                       [-1, 8,-1],
                       [-1,-1,-1]])

kernel_5x5 = np.array([[-1,-1,-1,-1,-1],
                       [-1, 1, 2, 1,-1],
                       [-1, 2, 4, 2,-1],
                       [-1, 1, 2, 1,-1],
                       [-1,-1,-1,-1,-1]])


k3 = ndimage.convolve(img,kernel_3x3)
k5 = ndimage.convolve(img,kernel_5x5)
g_hpf = img - cv2.GaussianBlur(img,(11,11),0) # Original - LPF
img_canny = cv2.Canny(img,200,300)


# fined edges with kernels
cv2.imshow("Original",img)
cv2.imshow("3x3",k3)
cv2.imshow("5x5",k5)
cv2.imshow("Gauss_hpf (Original-LPF)",g_hpf)
# fined edges with Canny
cv2.imshow("Canny",img_canny)

cv2.waitKey()
cv2.destroyWindow("3x3")
cv2.destroyWindow("5x5")
cv2.destroyWindow("Gauss_hpf (Original-LPF)")
cv2.destroyWindow("Canny")


# Sharpen, Passivation, Fuzzification, Embossed(Ridge)
kernel_3x3_positive = np.array([[-1,-1,-1],
                                [-1, 9,-1],
                                [-1,-1,-1]])

kernel_3x3_negative = np.array([[-1,-1,-1],
                                [-1, 7,-1],
                                [-1,-1,-1]])

kernel_5x5_blur = np.array([[0.04,0.04,0.04,0.04,0.04],
                            [0.04,0.04,0.04,0.04,0.04],
                            [0.04,0.04,0.04,0.04,0.04],
                            [0.04,0.04,0.04,0.04,0.04],
                            [0.04,0.04,0.04,0.04,0.04]])

kernel_3x3_embossed = np.array([[-2,-1, 0],
                                [-1, 1, 1],
                                [ 0, 1, 2]])

k3_positive = ndimage.convolve(img,kernel_3x3_positive)
k3_negative = ndimage.convolve(img,kernel_3x3_negative)
k5_blur = ndimage.convolve(img,kernel_5x5_blur)
k3_embossed = ndimage.convolve(img,kernel_3x3_embossed)

cv2.imshow("Sharpen with positive(3x3)",k3_positive)
cv2.imshow("Passivation with negative(3x3)",k3_negative)
cv2.imshow("Fuzzification with blur(5x5)",k5_blur)
cv2.imshow("Embossed(Ridge) with embossed(3x3)",k3_embossed)

cv2.waitKey()
cv2.destroyAllWindows()
