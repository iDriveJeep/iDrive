#!/usr/bin/env python

import numpy as np
import cv2

#libreria ros
import rospy
from std_msgs.msg import String

#convertidor entre ros y opencv
from cv_bridge import CvBridge, CvBridgeError


from sensor_msgs.msg import Image

lane_detector_topic = '/lane_detector/out_image'
zed_left_topic	    = '/zed/left/image_rect_color'

cv2_imge_output  = None
cv2_imge_warped = None
cv2_imge_window = None

def image_output_callback(msg1):
	cv2_imge_output = CvBridge().imgmsg_to_cv2(msg1, "bgr8")
	cv2.imshow("output",cv2_imge_output)
	cv2.waitKey(1)


def image_warped_callback(msg2):
	cv2_imge_warped = CvBridge().imgmsg_to_cv2(msg2, "mono8")
#	cv2.imshow("Warped",cv2_imge_warped)
#	cv2.waitKey(0)


def image_sliding_window_callback(msg3):
	cv2_imge_window = CvBridge().imgmsg_to_cv2(msg3, "bgr8")
#	cv2.imshow("sliding Window",cv2_imge_window)
#	cv2.waitKey(1)





if __name__ == '__main__':

	rospy.init_node('lane_detector_video', anonymous=True)

	
	rospy.Subscriber('/zed/left/image_rect_color', Image, image_output_callback,queue_size=2,buff_size=524288000)
	rospy.Subscriber('/lane_detector/warped_image', Image, image_warped_callback,queue_size=2,buff_size=524288000)
	rospy.Subscriber('/lane_detector/sliding_window_image', Image, image_sliding_window_callback,queue_size=2,buff_size=524280800)	
	
	
	
	try:
		if cv2_imge_output is not None:
			cv2.imshow("output",cv2_imge_output)
		if cv2_imge_warped is not None:
			cv2.imshow("Warped",cv2_imge_warped)
		if cv2_imge_window is not None:
			cv2.imshow("sliding Window",cv2_imge_window)
		cv2.waitKey(1)
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()
