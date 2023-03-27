#!/usr/bin/env python
import rospy
import roslib

from std_msgs.msg import String # for color
from nav_msgs.msg import Odometry # for pose
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from TiagoBears_ColorDetection.srv import Getcolor, GetcolorResponse, InitEmpty
from TiagoBears_ColorDetection.msg import StringArray

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

class ColorDetectorServer:
    """
    This class detect the per-cube color and count them

    """
    def __init__(self):
        self.image_topic = '/xtion/rgb/image_raw'
        # self.image_topic = '/bag/image_raw'
        self.bridge = CvBridge()
		
		# self.color_pubs = []
		# for i in range(28):
		# 	self.color_pubs.append(rospy.Publisher('/seen_colors'.format(i), String, queue_size=1))
        self.cubes=[]
		# self.cubesMsg=[]
        self.color_publish = rospy.Publisher('seen_colors', String, queue_size=10)
        self.s = rospy.Service('/get_colors', Getcolor, self.get_colors)
        self.init_empty_check=rospy.Service('/init_empty_check', InitEmpty, self.init)
        self.empty_left=rospy.Service('/empty_left', InitEmpty, self.check_left)

        self.init_left_img=None
        self.init_right_img=None

    def init(self, request):
        # get img
        img=rospy.wait_for_message(self.image_topic, Image)
        img=self.bridge.imgmsg_to_cv2(img, "bgr8")
        self.init_left_img=img[:200, :200]
        self.init_right_img=img[:200, -200:]
        return True

    def check_left(self, request):
        # get img
        img=rospy.wait_for_message(self.image_topic, Image)
        img=self.bridge.imgmsg_to_cv2(img, "bgr8")[:200, :200]
        diff = cv2.absdiff(img , self.init_left_img)
        cv2.imshow("diff",diff)
        cv2.waitKey(10000)
        return True
  
	# def update_colors(self):
	# 	""" A function to detect the color for the cube depedning on its position
  	# 	"""
	# 	img=rospy.wait_for_message(self.image_topic, Image)
	# 	img=self.bridge.imgmsg_to_cv2(img, "bgr8")
	# 	cube_color=[]
	# 	## you can write you code here to detect the color per-cube here

	# 	# publish the color
	# 	for i in range(28):
	# 		self.color_pubs[i].publish(cube_color[i])

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
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
                self.cubes.append([color,i,cX,cY])	
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
            #rospy.loginfo(color_and_locations[0][1])

        img_mask = self.color_detec(img, 'red')
        locations = self.search_contours(img,img_mask, 'red')
        rospy.loginfo(locations)
        cv2.imshow('image',img)
        cv2.waitKey(20000)

		# publish the color
        # for i in range(len(color_and_locations)):
        #     self.color_publish.publish(color_and_locations[i][0]+","+str(color_and_locations[i][1])+","+str(color_and_locations[i][2])+","+str(color_and_locations[i][3]))
        # self.color_publish.publish('***************************************************************')
        # color_and_locations=[]
    
    def get_colors(self,request):
        self.detect_cubes()
        print("_________________________")
        print("___Request color: "+request.req_color+"___")
        print("_________________________")
        red_cubes=[]
        blue_cubes=[]
        yellow_cubes=[]
        green_cubes=[]
        all_cubes=[]
        for pro_cube in self.cubes:
            #rospy.loginfo(pro_cube[0])
            if pro_cube[0]=='red':
                red_cubes.append(StringArray(pro_cube[0],pro_cube[1],pro_cube[2],pro_cube[3]))
            elif pro_cube[0]=='blue':
                blue_cubes.append(StringArray(pro_cube[0],pro_cube[1],pro_cube[2],pro_cube[3]))
            elif pro_cube[0]=='yellow':
                yellow_cubes.append(StringArray(pro_cube[0],pro_cube[1],pro_cube[2],pro_cube[3]))
            elif pro_cube[0]=='green':
                green_cubes.append(StringArray(pro_cube[0],pro_cube[1],pro_cube[2],pro_cube[3]))
            all_cubes.append(StringArray(pro_cube[0],pro_cube[1],pro_cube[2],pro_cube[3]))
        if request.req_color =="red":
            color_back = red_cubes
        elif request.req_color == "blue":
            color_back = blue_cubes
        elif request.req_color == "yellow":
            color_back = yellow_cubes
        elif request.req_color == "green":
            color_back = green_cubes
        elif request.req_color == "all":
            color_back = all_cubes
        else:
            color_wrong = StringArray('wrong color', 0,0,0)
            color_back = [color_wrong]
        response = GetcolorResponse()
        response.colors = color_back
        self.cubes=[]

        return response
	
