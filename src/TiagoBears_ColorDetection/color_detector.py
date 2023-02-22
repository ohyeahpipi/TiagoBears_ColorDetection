#!/usr/bin/env python
import rospy
import roslib

from std_msgs.msg import String # for color
from nav_msgs.msg import Odometry # for pose
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

color_H_range={
	'red':[0,10],
	'blue':[99,125],
	'yellow':[20,34],
	'green':[35,77]
}
color_Smin_Vmin={
	'red':[50,20],
	'blue':[25,25],
	'yellow':[50,20],
	'green':[50,20]
}

class ColorDetector:
	"""
	This class detect the per-cube color and count them

	"""
	def __init__(self):
		self.image_topic = '/xtion/rgb/image_raw'.format(id)
		self.bridge = CvBridge()
		
		self.color_pubs = []
		for i in range(28):
			self.color_pubs.append(rospy.Publisher('/seen_colors'.format(i), String, queue_size=1))
		self.cubes=[]
		self.cubesMsg=[]
		self.color_publish = rospy.Publisher('seen_colors', String, queue_size=10)
  
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

	def color_detec(self,img, color='red'):
		hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		lower_hue = color_H_range[color][0]
		upper_hue = color_H_range[color][1]
		lower_hsv = np.array([lower_hue, color_Smin_Vmin[color][0], color_Smin_Vmin[color][1]])
		upper_hsv = np.array([upper_hue, 255, 255])
		mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
		return mask

	def search_contours(self,img,mask,color):
		contours_count = 0
		_, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		i=0
		for contour in contours:
			area = cv2.contourArea(contour)
			if 5 < area < 10000:
				contours_count += 1
				M = cv2.moments(contour)
				if M["m00"] != 0:
					cX = int(M["m10"] / M["m00"])
					cY = int(M["m01"] / M["m00"])
				else:
					cX, cY = 0, 0
				cv2.circle(img, (cX, cY), 3, (255, 255, 255), -1)
				self.cubes.append({color+'{0}'.format(i):[cX,cY]})	
				i=i+1
		return self.cubes

	def detect_cubes(self):
		# get img
		img=rospy.wait_for_message(self.image_topic, Image)
		img=self.bridge.imgmsg_to_cv2(img, "bgr8")
		# get color and location
		for color in color_H_range.keys():
			img_mask = self.color_detec(img, color)
			color_and_locations = self.search_contours(img,img_mask, color)
			#rospy.loginfo(color_and_locations)

	###############################################for test#############
		# img_mask = self.color_detec(img, 'blue')
		# locations = self.search_contours(img,img_mask, 'blue')
		# rospy.loginfo(locations)

		# cv2.imshow('image',img)
		# cv2.waitKey(20000)
        #####################################################################

		# publish the color
		for i in range(len(color_and_locations)):
			for j in color_and_locations[i].items():
				self.color_publish.publish(j[0]+":"+str(j[1]))
		self.color_publish.publish('***************************************************************')
		rospy.Rate(1.0).sleep()
		



if __name__ == '__main__':
	rospy.init_node('color_detector')
	rospy.loginfo('color_detector node started')
	cd = ColorDetector()
	while not rospy.is_shutdown():
		cd.detect_cubes()
	#cd.detect_cubes()
	
