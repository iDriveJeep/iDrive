#!/usr/bin/env python

import numpy as np
import cv2
from math import hypot
import socket
import time
from functions.utils import *

#libreria ros
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

#convertidor entre ros y opencv
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

avgValue=[]

#ROI para recortes
top_left = 		(0, 0.5)
top_right = 	(0.8, 0.5)
bottom_left = 	(0, 1)
bottom_right = 	(.8, 1)
puntos = np.array([top_right,top_left,bottom_left,bottom_right])

#rango de valores de rojo en HSV
#rojo_bajos1 = np.array([0, 70, 50])
rojo_bajos1 = np.array([0, 50, 50])
rojo_altos1 = np.array([12, 255, 255])
rojo_bajos2 = np.array([160, 50, 50])
rojo_altos2 = np.array([188, 255, 255])

low_blue = np.array([94, 80, 2])
high_blue = np.array([126, 255, 255])
low_blue2 = np.array([270, 80, 2])
high_blue2 = np.array([281, 255, 255])

"""
#rango de valores de amarillo en HSV
amarillo_bajos1 = np.array([13, 50, 50])
amarillo_altos1 = np.array([60, 255, 255])
amarillo_bajos2 = np.array([189, 50, 50])
amarillo_altos2 = np.array([215, 255, 255])
"""

output_image_pub = rospy.Publisher('/lane_detector/out_image', Image, queue_size=2)

steer_msg = Float32()
error_lateral_msg = Float32()			#CAMBIOS

def image2_callback (msg):

	start_time = time.time()

	frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")
	
	height, width = frame.shape[:2]
	
	max_right_line = None
	max_left_line = None
	maxsizeL=0
	maxsizeR=0
	X=0
	
    #copia de frame 
	recorte=frame.copy()
	mask = np.zeros(recorte.shape[:2], np.uint8)
	
	roi_rojo = image_roi(frame, puntos)
	
	#cambio a HSV
	hsv = cv2.cvtColor(roi_rojo, cv2.COLOR_BGR2HSV)
	
	#definir kernel para operaciones de erosion y dilatacion
	kernel = np.ones((5,5),np.uint8)
	
	#mascara de rojos

	mask1 = cv2.inRange(hsv, rojo_bajos1, rojo_altos1)
	mask2 = cv2.inRange(hsv, rojo_bajos2, rojo_altos2)
	#mask1 = cv2.inRange(hsv, low_blue,high_blue)
	#mask2 = cv2.inRange(hsv, low_blue,high_blue)
	#or mascaras de rojos
	rojos = cv2.add(mask1, mask2)
	
	#erosion de mascara de rojos y dilatacion
	rojos = cv2.erode(rojos, kernel, iterations = 1)
	rojos = cv2.dilate(rojos, kernel, iterations = 1)
	
	warped = perspective_warp(rojos, dst_size=(frame.shape[1], frame.shape[0]))
	
	#rotation = avg_angles(warped)
	
	"""
	if rotation > 20:
		M = cv2.getRotationMatrix2D((height/2,width/2), -rotation,1)
		warped = cv2.warpAffine(warped,M,(width,height))
	"""
	out_img, curves, lanes, ploty = sliding_window(warped, nwindows=25, margin=18)
	
	img_ , steering_angle, error_lateral = draw_lanes(frame, curves[0], curves[1])
	#print('error',error_lateral)
	#print('steering',steering_angle)
	
	###########################################
	steer_msg.data = steering_angle
	pub_steer.publish(steer_msg)	
	error_lateral_msg.data = error_lateral		#CAMBIO
	pub_error_lat.publish(error_lateral_msg)	#CAMBIO
	###########################################
	
	numpy_horizontal_concat = np.concatenate((out_img, img_), axis = 1)
	
	cv2.imshow("filtro rojos", rojos)
	cv2.imshow("Filtros", numpy_horizontal_concat)
	print("-----%s segundo de ejecuccion    ----" % (time.time()-start_time))	
	
	cv2.waitKey(1)

if __name__ == '__main__':

	rospy.init_node('lane_testing_2', anonymous=True)

	
	rospy.Subscriber(rospy.get_param("~subscriber_topic_2"), Image, image2_callback,queue_size=2,buff_size=52428800)	
	pub_steer = rospy.Publisher (rospy.get_param("~subscriber_tropic_steer"),Float32,queue_size = 10)
	pub_error_lat = rospy.Publisher (rospy.get_param("~subscriber_topic_error_lat"),Float32,queue_size = 10)			#CAMBIO
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")	
		cv2.destroyAllWindows()



