#!/usr/bin/python
import roslib
import rospy
import tf
import math
import random
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import time

global joint_control

joint_control = rospy.Publisher('/joint_states_ct',JointState,queue_size=10) # Publish Joint States

def move(arr):
    global joint_control
    cmd = JointState()
    cmd.name = ['Joint_1','Joint_2','Joint_3']
    cmd.position = list(arr) # Robot Home POSITION
    joint_control.publish(cmd)

def look_around_ccw(m,u=1.57):
    delay = 0.1
    time.sleep(0.8)
    move([0,m,u])
    angles = np.linspace(-1.57,1.57,180)
    for i in angles:
        move([i,m,u])
        time.sleep(delay)


def look_around_cw(m,u=1.57):
    delay = 0.1
    time.sleep(0.8)
    move([0,m,u])
    angles = np.linspace(-1.57,1.57,180)
    angles = np.flip(angles)
    for i in angles:
        move([i,m,u])
        time.sleep(delay)

def look_around_m():
    m_range = np.linspace(-1.25,1.25,10)
    u = 1.57
    for i in range(len(m_range)):
        # if m_range[i]<100:
        if i%2 == 0:
            look_around_ccw(m_range[i])
        else:
            look_around_cw(m_range[i])
        # else:
        #     if i%2 == 0:
        #         look_around_ccw(servo,m_range[i],m_range[i])
        #     else:
        #         look_around_cw(servo,m_range[i],180-m_range[i])


def robot_home():
    move([0,0,0])


if __name__=='__main__':
    rospy.init_node('look_around')

    rate = rospy.Rate(10.0)
    robot_home()
    look_around_m()
    robot_home()
    # while not rospy.is_shutdown():
    #     # cmd.data = [90,90,90]
    #     # servo.publish(cmd)
    #     rate.sleep()