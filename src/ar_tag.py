import cv2 as cv2
import numpy as np

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)

arucoParams = cv2.aruco.DetectorParameters_create()

img = cv2.imread('Aruco_tag.jpg')

(corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

# cv2.imshow('img',img)
# cv2.waitKey(0)

print(corners)
print(ids)