#!/usr/bin/env python3


from time import sleep
import cv_bridge
from sensor_msgs.msg import Image
from rover_nav.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from aruco_library import *
import math


class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('marker_detection') #Initialise rosnode 
		
		# Making a publisher 
		
		self.marker_pub = rospy.Publisher('/marker_info', Marker, queue_size=1)
		self.img_pub = rospy.Publisher('/live_detection/images', Image, queue_size=1)
		# ------------------------Add other ROS Publishers here-----------------------------------------------------

		self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		
		# -------------------------Add other ROS Subscribers here----------------------------------------------------
        
		self.img = np.empty([]) #
		self.bridge = CvBridge()
		
		self.marker_msg = Marker()  # This will contain the message structure of message type task_1/Marker

		self.detection_dict = {}
		self.angle_dict  = {}
		self.rate = rospy.Rate(10)

	# Callback function of amera topic
	def image_callback(self, data):
	# Note: Do not make this function lenghty, do all the processing outside this callback function
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			self.get_info()

		except CvBridgeError as e:
			print(e)
			return

	def publish_data(self):
		self.marker_pub.publish(self.marker_msg)

	def get_info(self):
		self.detection_dict = detect_ArUco(self.img)							
		self.angle_dict = Calculate_orientation_in_degree(self.detection_dict)
		self.publish_detection_image()

	def publish_detection_image(self):
		detection_image = mark_ArUco(self.img,self.detection_dict ,self.angle_dict)
		detection_msg = self.bridge.cv2_to_imgmsg(detection_image)
		self.img_pub.publish(detection_msg)
		self.execute()


	def execute(self):
		try:
			for i in self.detection_dict:
				x_red = (self.detection_dict[i][0][0] + self.detection_dict[i][1][0] + self.detection_dict[i][2][0] + self.detection_dict[i][3][0])/4
				y_red = (self.detection_dict[i][0][1] + self.detection_dict[i][1][1] + self.detection_dict[i][2][1] + self.detection_dict[i][3][1])/4
				
				self.marker_msg.id = i
				self.marker_msg.x = x_red 
				self.marker_msg.y = y_red

				# self.marker_msg.yaw = math.radians(self.angle_dict[i])
				self.marker_msg.yaw = self.angle_dict[i]
				
				self.publish_data()
				self.rate.sleep()
		except TypeError:
			pass



if __name__ == '__main__':
	image_proc_obj = image_proc()
	rospy.spin()
