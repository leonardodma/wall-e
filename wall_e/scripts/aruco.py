#! /usr/bin/env python
# -*- coding:utf-8 -*-
import cv2
import cv2.aruco as aruco
import numpy as np 
import sys 

import os


def identifica_id(cv_image):
	gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

	# --- Define the aruco dictionary
	aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
	# parameters  = aruco.DetectorParameters_create()
	# parameters.minDistanceToBorder = 0
	# parameters.adaptiveThreshWinSizeMax = 1000

	corners, ids, rejectedImgPoints = aruco.detectMarkers(
		gray, aruco_dict)  # , parameters=parameters)


	aruco.drawDetectedMarkers(cv_image, corners, ids)

	if ids is None:
		ids = [[0]]
		centro = (0,0)

	else:
		ids = ids.tolist()
		corners = corners[0][0]
		media_x = int((corners[0][0]+corners[1][0]+corners[2][0]+corners[3][0])/4)
		media_y = int((corners[0][1]+corners[1][1]+corners[2][1]+corners[3][1])/4)
		centro = (media_x, media_y)

	return centro, ids[0][0], ids[0]
