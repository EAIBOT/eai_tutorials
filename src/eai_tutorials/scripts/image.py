#!/usr/bin/env python
import cv2
import numpy
import os

# create_image

randomByteArray = bytearray(os.urandom(120000))
flatNumpyArray = numpy.array(randomByteArray)
flatNumpyArray = numpy.random.randint(0,256,120000)

grayImage = flatNumpyArray.reshape(300,400)
bgrImage = flatNumpyArray.reshape(100,400,3)

cv2.imwrite('../images/RandomGray.jpg',grayImage)
cv2.imwrite('../images/RandomColor.jpg',bgrImage)

cv2.imshow('RandomGray.jpg',cv2.imread('../images/RandomGray.jpg'))
cv2.imshow('RandomColor.jpg',cv2.imread('../images/RandomColor.jpg'))

cv2.waitKey()
cv2.destroyAllWindows()


# numpy_array

img = cv2.imread('../images/flower.jpg')
print 'shape: ',img.shape
print 'size : ',img.size
print 'dtype: ',img.dtype

print 'img.item(10,120,0) value: ',img.item(10,120,0)
img.itemset((10,120,0),255)
print 'img.item(10,120,0) value: ',img.item(10,120,0)

cv2.imshow('flower.jpg',img)
cv2.waitKey()
cv2.destroyAllWindows()


# Region of Interest

img = cv2.imread('../images/flower.jpg')
roi = img[0:100, 0:100]
img[100:200, 100:200] = roi
cv2.imshow('ROI', img)

cv2.waitKey()
cv2.destroyAllWindows()


# no_green

img = cv2.imread('../images/flower.jpg')
cv2.imshow('Original', img)

img[:, :, 1] = 0
cv2.imshow('no_Green', img)

cv2.waitKey()
cv2.destroyAllWindows()
