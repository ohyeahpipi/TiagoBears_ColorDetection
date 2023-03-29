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
    'blue':[100,124],
    'yellow':[20,34],
    'green':[35,90]
}
color_Smin_Vmin={
    'red':[100,100],
    'blue':[40,80],
    'yellow':[60,240],
    'green':[40,140]
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
        self.left_cube_color = "no color info"
        self.right_cube_color = "no color info"
        self.left_mask1=None
        self.right_mask1=None
		# self.cubesMsg=[]
        self.color_publish = rospy.Publisher('seen_colors', String, queue_size=10)
        self.s = rospy.Service('/TiagoBears/get_colors', Getcolor, self.get_colors)
        self.init_empty_check=rospy.Service('/TiagoBears/init_empty_check', InitEmpty, self.init)
        self.empty_left=rospy.Service('/TiagoBears/is_empty_left', InitEmpty, self.check_left)
        self.empty_right=rospy.Service('/TiagoBears/is_empty_right', InitEmpty, self.check_right)

        self.init_left_img=None
        self.init_right_img=None

    def init(self, request):
        # get img
        img=rospy.wait_for_message(self.image_topic, Image)
        img=self.bridge.imgmsg_to_cv2(img, "bgr8")
        # the init_img should be reset every time 
        self.init_left_img=img[:200, :200]
        self.init_right_img=img[:200, -200:]
        print("init_img are saved")
        return True

    def check_left(self, request):
        # get img
        img=rospy.wait_for_message(self.image_topic, Image)
        img=self.bridge.imgmsg_to_cv2(img, "bgr8")[:200, :200]
        diff = cv2.absdiff(img , self.init_left_img)
        thresh1 = self.get_mask(diff)
        # print(np.sum(diff))
        # cv2.imshow("diff",diff)
        # cv2.imshow("thresh",thresh1)
        # cv2.waitKey(10000)
        return not (np.sum(diff) > 700000) # true means: gripper is empty, no cube found

    def check_right(self,request):
        img=rospy.wait_for_message(self.image_topic, Image)
        img=self.bridge.imgmsg_to_cv2(img, "bgr8")[:200, -200:]
        diff = cv2.absdiff(img , self.init_right_img)
        # cv2.imshow("diff",diff)
        # cv2.waitKey(10000)
        return not (np.sum(diff) > 700000) # true means: gripper is empty, no cube found

    def get_mask(self,diff_img):
        lower_hsv=np.array([0,43,46])
        upper_hsv=np.array([180,255,255])
        hsv=cv2.cvtColor(diff_img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, lower_hsv, upper_hsv)
        return mask1
    
    def cut_img(self,img,mask1):
        result_img = cv2.bitwise_and(img,img,mask1)
        return result_img
        
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

    def search_contours(self,img,mask,color,arm_flag):
        contours_count = 0
        _,contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if 50 < area < 10000:
                contours_count += 1
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0
                cv2.circle(img, (cX, cY), 3, (255, 255, 255), -1)
                if arm_flag == "left":
                    self.left_cube_color = color
                elif arm_flag == "right":
                    self.right_cube_color = color
                #rospy.loginfo(color)


    def detect_cubes(self):
        # get img
        img=rospy.wait_for_message(self.image_topic, Image)
        img=self.bridge.imgmsg_to_cv2(img, "bgr8")
        current_left_img = img[:200, :200]
        current_right_img = img[:200, -200:]

        # ##get color and location
        for color in color_H_range.keys():
            arm_flag = "left"
            diff = cv2.absdiff(current_left_img, self.init_left_img)
            thresh1 = self.get_mask(diff)
            c_img = self.cut_img(current_left_img,thresh1)
            img_mask = self.color_detec(c_img, color)
            self.search_contours(c_img,img_mask, color,arm_flag)
            arm_flag = "right"
            diff = cv2.absdiff(current_right_img, self.init_right_img)
            thresh1 = self.get_mask(diff)
            c_img = self.cut_img(current_right_img,thresh1)
            img_mask = self.color_detec(c_img, color)
            self.search_contours(c_img,img_mask, color,arm_flag)
        rospy.loginfo(self.left_cube_color)
        rospy.loginfo(self.right_cube_color)


        # diff = cv2.absdiff(current_left_img, self.init_left_img)
        # thresh1 = self.get_mask(diff)
        # c_img = self.cut_img(current_left_img,thresh1)
        # img_mask = self.color_detec(c_img, 'green')
        # self.search_contours(c_img,img_mask, 'green',"left")
        # rospy.loginfo(self.left_cube_color)
        # cv2.imshow('mask_image',img_mask)
        # cv2.imshow('img',current_left_img)
        # cv2.waitKey(20000)

	# publish the color
        # for i in range(len(color_and_locations)):
        #     self.color_publish.publish(color_and_locations[i][0]+","+str(color_and_locations[i][1])+","+str(color_and_locations[i][2])+","+str(color_and_locations[i][3]))
        # self.color_publish.publish('***************************************************************')
        # color_and_locations=[]
    
    def get_colors(self,request):
        self.detect_cubes()
        print("_________________________")
        print("___Request color: "+request.req_arm+"___")
        print("_________________________")
        left_arm_cube = self.left_cube_color
        right_arm_cube = self.right_cube_color
        both_arm_cubes=[]
        both_arm_cubes.append(StringArray("left_arm",left_arm_cube))
        both_arm_cubes.append(StringArray("right_arm",right_arm_cube))


        if request.req_arm =="left":
            color_back = [StringArray("left_arm",left_arm_cube)]
        elif request.req_arm == "right":
            color_back = [StringArray("right_arm",right_arm_cube)]
        elif request.req_arm == "both":
            color_back = both_arm_cubes
        else:
            color_wrong = StringArray("wrong_arm_req", "no color info")
            color_back = [color_wrong]
        response = GetcolorResponse()
        response.colors = color_back
        self.left_cube_color = "no color info"
        self.right_cube_color = "no color info"

        return response
	
