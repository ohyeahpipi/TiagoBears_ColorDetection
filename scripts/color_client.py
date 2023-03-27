#!/usr/bin/env python
import rospy
import sys
from TiagoBears_ColorDetection.srv import *
from TiagoBears_ColorDetection.msg import *

def get_color_client(data):
    rospy.wait_for_service('/get_colors')
    try:
        get_color = rospy.ServiceProxy('/get_colors',Getcolor)
        resp1 = get_color(data)
        return resp1.colors
    except rospy.ServiceException as e:
        print("Service call failed:%s"%e)

def usage():
    return "%s [left/right/both]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv)==2:
        data = sys.argv[1]
    else:
        print(usage())
        sys.exit(1)
    print("Requesting get_color...")
    a = get_color_client(data)
    for i in a:
        print("_____")
        print(i)