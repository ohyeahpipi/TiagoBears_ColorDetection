#!/usr/bin/env python   
import rospy
from TiagoBears_ColorDetection.color_detector import ColorDetectorServer
rospy.init_node('ColorDetector', anonymous=True)
color_detector = ColorDetectorServer()
color_detector.detect_cubes()