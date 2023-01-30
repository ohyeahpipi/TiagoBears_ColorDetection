#!/usr/bin/env python
import rospy
import roslib

from std_msgs.msg import String # for color
from nav_msgs.msg import Odometry # for pose
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

class ColorDetector:
	"""
	This class detect the per-cube color and count them

	"""
	def __init__(self):
		self.image_topic = '/xtion/rgb/image_raw'.format(id)
		self.bridge = CvBridge()
		
		self.color_pubs = []
		for i in range(28):
			self.color_pubs.append(rospy.Publisher('/cube_{0}_color'.format(i), String, 1))
  
	def update_colors(self):
		""" A function to detect the color for the cube depedning on its position
  		"""
		img=rospy.wait_for_message(self.image_topic, Image)
		img=self.bridge.imgmsg_to_cv2(img, "bgr8")
		cube_color=[]
		## you can write you code here to detect the color per-cube here

		# publish the color
		for i in range(28):
			self.color_pubs[i].publish(cube_color[i])