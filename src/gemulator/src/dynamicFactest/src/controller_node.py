#!/usr/bin/env python
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String

from math import *
from numpy.linalg import norm
import numpy as np

# from planner_node import get_list

processing = False
new_msg = False
msg = None

def callback(data):
    global processing, new_msg, msg
    if not proccessing:
        new_msg = True
        msg = data

def listener():
    global processing, new_msg, msg
    rospy.init_node('listener')
    rospy.Subscriber('path', numpy_msg(Floats), callback)
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        if new_msg:
            processing = True
            new_msg = False
            print('simulating process')
            rospy.loginfo(msg)
            r.sleep()
            processing = False

# pt_list = []
#
#
# def callback(data):
#     # Get the new path
#     global pt_list
#     pt_list = data.data
#     pt_list = pt_list.reshape(len(pt_list)/2, 2)
#
# def listener():
#     rospy.init_node('listener')
#     a = rospy.Subscriber("path", numpy_msg(Floats), callback)
#     rospy.spin()
#
def main():
    # Get subscriber data
    # listener()
    while 1:
        print(get_list())

if __name__ == '__main__':
    # main()
    listener()
