#!/usr/bin/env python
import rospy

from std_msgs.msg import String # for color
from nav_msgs.msg import Odometry # for pose

class ColorDetector:
	"""
	This class detect the per-cube color and count them

	"""
	def __init__(self, id):
		self.id=id
		self._stacked = False
  
		self.pose = None
		self._pose_topic = '/cube_{0}_odom'.format(id)
		# continuously update the pose:
		# subscribe to the pose estimation topic
		self.pose_sub=rospy.Subscriber(self._pose_topic, Odometry, self.update_pose)
		
		self._color_topic='/cube_{0}_color'.format(id)
		# publish the color of each cube
		self.color_pub=rospy.Publisher(self._color_topic, String, 1)

	def update_pose(self, msg: Odometry):
		""" A callback function to update the pose whenever the pose subscriber recieves a topic
  		"""
		self.pose = msg.pose.pose
  
	def detect_color(self):
		""" A function to detect the color for the cube depedning on its position
  		"""