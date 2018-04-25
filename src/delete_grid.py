#!/usr/bin/env python

import cv2
import numpy as np

img = cv2.imread('grid.png', 0)
print img.shape
kernel = np.ones((4,4),np.uint8)
ret,thresh = cv2.threshold(img,127,255,cv2.THRESH_BINARY_INV)
opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
#erosion = cv2.erode(thresh,kernel,iterations = 1)
#print cv2.HoughLinesP(opening, 1, 1, 1)
print opening[1][1]

cv2.imwrite('gridless.png' ,opening)
cv2.imshow('img',opening)
cv2.waitKey(0)
cv2.destroyAllWindows()

