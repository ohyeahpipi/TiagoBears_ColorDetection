#!/usr/bin/env python   
import rospy
from TiagoBears_ColorDetection.color_detector import ColorDetector
rospy.init_node('ColorDetector', anonymous=True)
color_detector = ColorDetector()
color_detector.update_colors()