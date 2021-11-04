#!/usr/bin/python3
############## Task1.1 - ArUco Detection ##############


import numpy as np
import cv2
from numpy.core.records import array
import cv2.aruco as aruco
import sys
import math
import time

# Please check the output of this function values are float where as we require int values
def detect_ArUco(img):
	## function to detect ArUco markers in the image using ArUco library
	## argument: img is the test image
	## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
	## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
	## 				{0: array([[315, 163],
	#							[319, 263],
	#							[219, 267],
	#							[215,167]], dtype=float32)}

	Detected_ArUco_markers = {}
    ## enter your code here ##
    
	# aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
	
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	parameters = aruco.DetectorParameters_create()

	parameters.minDistanceToBorder =  1
	parameters.cornerRefinementMaxIterations = 149
	parameters.minOtsuStdDev= 4.0
	parameters.adaptiveThreshWinSizeMin= 3
	parameters.adaptiveThreshWinSizeStep= 49
	parameters.minMarkerDistanceRate= 0.0149
	parameters.maxMarkerPerimeterRate= 10.075 
	parameters.minMarkerPerimeterRate= 0.2524 
	parameters.polygonalApproxAccuracyRate= 0.0556
	parameters.cornerRefinementWinSize= 9
	parameters.adaptiveThreshConstant= 9.0
	parameters.adaptiveThreshWinSizeMax= 369
	parameters.minCornerDistanceRate= 0.091



	corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
	try:
		for i in range(len(ids)):
			Detected_ArUco_markers[int(ids[i])] = corners[i][0]
		# print(len(Detected_ArUco_markers))
		return Detected_ArUco_markers
	except ValueError and TypeError:
		# value error for list and Type error if ids is None 
		pass

def Calculate_orientation_in_degree(Detected_ArUco_markers):
	## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
	## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
	## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
	##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
	##			function should return: {1: 120 , 2: 164}



	ArUco_marker_angles = {}
	## enter your code here ##
	try:
		for i in Detected_ArUco_markers:
			x_of_midpoint = (Detected_ArUco_markers[i][0][0] + Detected_ArUco_markers[i][1][0])/2
			y_of_midpoint = (Detected_ArUco_markers[i][0][1] + Detected_ArUco_markers[i][1][1])/2
			# print(x_of_midpoint)


			x_red = (Detected_ArUco_markers[i][0][0] + Detected_ArUco_markers[i][1][0] + Detected_ArUco_markers[i][2][0] + Detected_ArUco_markers[i][3][0])/4
			y_red = (Detected_ArUco_markers[i][0][1] + Detected_ArUco_markers[i][1][1] + Detected_ArUco_markers[i][2][1] + Detected_ArUco_markers[i][3][1])/4



			# Transformation from 0,0 to x_red, y_red

			new_mid_x = x_of_midpoint - x_red
			new_mid_y = y_of_midpoint - y_red

			# print(x_red, y_red)
			# h, w, c = img.shape

			# print(new_mid_x, new_mid_y, i)

			slope = new_mid_y/new_mid_x
			if new_mid_y >= 0 and new_mid_x <= 0:             # correct angle between 180 -270 
				ArUco_marker_angles[i] = 180 + abs(int(math.degrees(math.atan(slope))))
			elif new_mid_y >= 0 and new_mid_x >= 0:          	# Correct angle between 270 - 360 
				ArUco_marker_angles[i] = 360 - abs(int(math.degrees(math.atan(slope))))
			elif new_mid_y <= 0 and new_mid_x <= 0:    		# Correct angle between 90 - 180
				ArUco_marker_angles[i] = 180 - abs(int(math.degrees(math.atan(slope))))
			elif new_mid_y <= 0 and new_mid_x >= 0:        	# correct angle between 0 - 90
				ArUco_marker_angles[i] = abs(int(math.degrees(math.atan(slope))))
			# print(slope)
	except TypeError:
		pass

	return ArUco_marker_angles	## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
	## function to mark ArUco in the test image as per the instructions given in problem statement
	## arguments: img is the test image 
	##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
	##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
	## return: image namely img after marking the aruco as per the instruction given in problem statement
	
    ## enter your code here ##

	# lets make dots first
	radius = 8
	thickness = -1
	# BGR
	color = [
		(100, 100, 100), 	# grey
		(0, 250, 0),	# Green
		(180, 105, 250), # Pink
		(255, 255, 255) # white
		]
	red = (0, 0, 255) # RED
	x_red, y_red = 0, 0
	try:
		for id in Detected_ArUco_markers:
			# for idx, val in enumerate(Detected_ArUco_markers[id]):
			temp_val = Detected_ArUco_markers[id].tolist()

			img = cv2.circle(img,(int(temp_val[0][0]), int(temp_val[0][1])), radius, color[0], thickness)
			img = cv2.circle(img,(int(temp_val[1][0]), int(temp_val[1][1])), radius, color[1], thickness)
			img = cv2.circle(img,(int(temp_val[2][0]), int(temp_val[2][1])), radius, color[2], thickness)
			img = cv2.circle(img,(int(temp_val[3][0]), int(temp_val[3][1])), radius, color[3], thickness)

			x_red = int(temp_val[0][0] + temp_val[1][0] + temp_val[2][0] + temp_val[3][0])//4
			y_red = int(temp_val[0][1] + temp_val[1][1] + temp_val[2][1] + temp_val[3][1])//4
			img = cv2.circle(img, (x_red, y_red), radius, red, thickness)

			x_of_midpoint = int(temp_val[0][0] + temp_val[1][0])//2
			y_of_midpoint = int(temp_val[0][1] + temp_val[1][1])//2
			
			img = cv2.line(img, (x_red, y_red), (x_of_midpoint, y_of_midpoint), (200,0, 0) ,thickness=3)
			font = cv2.FONT_HERSHEY_SIMPLEX
			# print(x_red, y_red)
			
			
			img = cv2.putText(img, str(id), (x_red, y_red), font, fontScale=1, color=red, thickness=2)
			img = cv2.putText(img, str(ArUco_marker_angles[id]), (int(temp_val[3][0]), int(temp_val[3][1])), font, fontScale=1, color=(50, 250, 50), thickness=2)

	except TypeError:
		pass



	return img