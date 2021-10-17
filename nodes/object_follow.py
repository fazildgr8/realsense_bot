#!/usr/bin/python
from pickle import TRUE
import roslib
import rospy
import tf
import math
import random
from darknet_ros_msgs.msg import BoundingBoxes,ObjectCount
import numpy as np
from rospy.numpy_msg import numpy_msg
import time
from sensor_msgs.msg import JointState
import threading
import cv2

global obj,loc,cnt,rect,flag
global joint_control,curr


curr = [0,0,0]
obj = rospy.get_param('class','bottle')

rect = None
loc = [0,0]
cnt = 0
flag = False

def callback(msg):
    global obj,loc,cnt,rect,flag
    bounding_boxes = msg.bounding_boxes
    if(cnt>0):
        for box in bounding_boxes:
            if(box.Class==obj):
                flag = True
                xmin = box.xmin
                ymin = box.ymin
                xmax = box.xmax
                ymax = box.ymax
                rect = [xmin,ymin,xmax,ymax]
                x = rect[2] - (rect[2]-rect[0])/2
                y = rect[3] - (rect[3]-rect[1])/2
                loc = [x,y]
            else:
                flag = False
    # print(obj,' loc = ',loc)
    
def object_count(msg):
    global cnt
    cnt = msg.count

def curr_pos(msg):
    global curr
    msg.position = curr

if __name__=='__main__':
    rospy.init_node('object_follower')
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback) 
    rospy.Subscriber("/darknet_ros/found_object", ObjectCount, object_count)
    rospy.Subscriber("/joint_states", JointState, curr_pos)
    joint_control = rospy.Publisher('/joint_states_ct',JointState,queue_size=10) # Publish Joint States
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        center_x = 640/2
        center_y = 480/2
        x = loc[0]
        y = loc[1]
        ex = center_x-x
        ey = center_y-y
        b = ex*0.0001
        m = -ey*0.0001
        arr = curr
        curr[0] = curr[0]+b
        curr[1] = curr[1]+m

        print(curr)
        if(abs(ex)<20 and abs(ey)<20):
            print('Centered')
        else:

            cmd = JointState()
            cmd.name = ['Joint_1','Joint_2','Joint_3']
            cmd.position = curr
            joint_control.publish(cmd)
        time.sleep(0.2)
    rospy.spin()