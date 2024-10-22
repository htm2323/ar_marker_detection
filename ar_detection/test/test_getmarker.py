import cv2
import cv2.aruco as aruco
import numpy as np

# script for check ar marker image

# dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
# dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
# dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_7X7_50)
dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_7X7_1000)
file_name = 'ar_marker_7x7_1000_1.png'
generator = aruco.drawMarker(dict_aruco, 0, 150)
cv2.imwrite(file_name, generator)

cv2.imshow('ArMarker', generator)
cv2.waitKey(1000)