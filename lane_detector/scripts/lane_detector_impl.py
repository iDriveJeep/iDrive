#!/usr/bin/env python

#libreria de funciones
from functions.utils import *

import numpy as np
import cv2

#libreria ros
import rospy
from std_msgs.msg import String

#convertidor entre ros y opencv
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

red = (0, 0, 255)
blue = (255, 0, 0)
green = (0, 255, 0)
kernel = np.ones((3,3),np.uint8)
camara = 'project_video.mp4'

#cap = cv2.VideoCapture(camara)
image_topic = "/videofile/image_raw"
#lectura de mensaje

output_image_pub = rospy.Publisher('/lane_detector/out_image', Image, queue_size=2)
warped_image_pub = rospy.Publisher('/lane_detector/warped_image', Image, queue_size=2)
sliding_window_pub = rospy.Publisher('/lane_detector/sliding_window_image', Image, queue_size=2)


def image_callback(msg):
	
	cv2_imge = CvBridge().imgmsg_to_cv2(msg, "bgr8")
	#cv2.imshow("Original",cv2_imge)
	# lectura de camara
	#ret, frame = cap.read()
	
	#if not ret:
	#	print( "Error en la lectura de imagen" )
	#	cap = cv2.VideoCapture(camara)
		#ret, frame = cap.read()
	
	# width, height, channel = frame.shape
	width = 640
	height = 480
	#cv2_imge = cv2.resize(cv2_imge, (width, height))
	midpoint = int(cv2_imge.shape[1]/2)
	# img = cv2.imread('Curved-Lane-Lines/test_images/test3.jpg')

	img = cv2.cvtColor(cv2_imge, cv2.COLOR_BGR2RGB)
	dst = pipeline(img)

	# #ajustes morfologicos
	# dst = cv2.morphologyEx(dst, cv2.MORPH_OPEN, kernel)
	# dst = cv2.morphologyEx(dst, cv2.MORPH_CLOSE, kernel)

	dst = perspective_warp(dst, dst_size=(dst.shape[1], dst.shape[0]))

	out_img, curves, lanes, ploty = sliding_window(dst, margin=int(cv2_imge.shape[1]*.1))
	curverad=get_curve(img, curves[0],curves[1])
	img_ = draw_lanes(cv2_imge, curves[0], curves[1])

	# #midpoint 
	cv2.line(img_, (midpoint, int(cv2_imge.shape[0]*0.9)),(midpoint, int(cv2_imge.shape[0]*0.9)), red, 8)

	cv2.imshow("out", out_img)
	cv2.imshow("hola",dst)
	cv2.imshow("image", img_)
	
	cv2.waitKey(1)

	output_msg = CvBridge().cv2_to_imgmsg(img_,"bgr8")
	warped_msg = CvBridge().cv2_to_imgmsg(dst,"mono8")
	sliding_window_msg = CvBridge().cv2_to_imgmsg(out_img,"bgr8")

	output_image_pub.publish(output_msg)
	warped_image_pub.publish(warped_msg)
	sliding_window_pub.publish(sliding_window_msg)

 
if __name__ == '__main__':

	rospy.init_node('lane_detector_node', anonymous=True)

	
	rospy.Subscriber(rospy.get_param("~subscriber_topic_2"), Image, image_callback,queue_size=2,buff_size=52428800)	

	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

