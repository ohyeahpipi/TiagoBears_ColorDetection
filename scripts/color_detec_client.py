#!/usr/bin/env python

import sys
import rospy
from TiagoBears_ColorDetection.srv import *
from TiagoBears_ColorDetection.msg import *

def get_colors_client(data):
    rospy.wait_for_service('/get_colors')
    try:
        get_color_coords = rospy.ServiceProxy('/get_colors', Getcolor)
        resp1 = get_color_coords(data)
        return resp1.colors
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [red/blue/yellow/green/all]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        data = sys.argv[1]
    else:
        print(usage())
        sys.exit(1)
    print("Requesting get_colors...")
    #print(fetch_topic_client("place_holder"))
    a = get_colors_client(data)
    #print(a[1].color)
    #print(a)
    for i in a:
        print("_____")
        print(i)