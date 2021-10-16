#!/usr/bin/python
from numpy.lib.function_base import angle
import roslib
import rospy
import tf
import math
import random
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import time


def look_around_ccw(joint_control,m=0,u=0):
    cmd = JointState()
    cmd.position = [0,m,u]
    cmd.name = ['Joint_1','Joint_2','Joint_3']
    delay = 0.1
    time.sleep(0.8)
    joint_control.publish(cmd)
    angles = np.linspace(-1.57,1.57,180)
    for i in range(len(angles)):
        cmd.position = [angles[i],m,u]
        joint_control.publish(cmd)
        time.sleep(delay)


def look_around_cw(joint_control,m=0,u=0):
    cmd = JointState()
    cmd.name = ['Joint_1','Joint_2','Joint_3']
    cmd.position = [0,m,u]
    delay = 0.1
    time.sleep(0.8)
    joint_control.publish(cmd)
    angles = np.linspace(-1.57,1.57,180)
    angles = np.flip(angles,0)

    for i in range(len(angles)):
        cmd.position = [angles[i],m,u]
        joint_control.publish(cmd)
        time.sleep(delay)

def map_sequence(joint_control):
    m_range = np.linspace(1.4,-1.4,10)
    for i in range(len(m_range)):
        if i%2==0:
            look_around_ccw(joint_control,m_range[i])
            look_around_cw(joint_control,m_range[i])
        else:
            look_around_ccw(joint_control,m_range[i])
            look_around_cw(joint_control,m_range[i])

def robot_home(joint_control):
    cmd = JointState()
    cmd.name = ['Joint_1','Joint_2','Joint_3']
    cmd.position = [0,0,0] # Robot Home POSITION
    joint_control.publish(cmd)


if __name__=='__main__':
    rospy.init_node('look_around')

    rate = rospy.Rate(10.0)
    cmd = JointState()
    
    joint_control = rospy.Publisher('/joint_states_ct',JointState,queue_size=10) # Publish Joint States
    cmd.name = ['Joint_1','Joint_2','Joint_3']
    cmd.position = [0,0,0] # Robot Home POSITION
    # time.sleep(5)
    joint_control.publish(cmd)
    map_sequence(joint_control)
    robot_home(joint_control)
    # while not rospy.is_shutdown():
        # cmd.data = [90,90,90]
        # servo.publish(cmd)
        # joint_control.publish(cmd)
        # rate.sleep()
    # rospy.spin()