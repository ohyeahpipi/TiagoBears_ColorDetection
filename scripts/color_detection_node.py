#!/usr/bin/env python   
import rospy
from TiagoBears_ColorDetection.color_detector import ColorDetectorServer
rospy.init_node('ColorDetector', anonymous=True)
color_detector = ColorDetectorServer()
color_detector.init(False)
# input("cube")
# color_detector.check_left(False)
rospy.spin()
color_detector.detect_cubes()