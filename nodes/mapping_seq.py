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



def look_updown(joint_control,b=0,start=0,d='up'):
    cmd = JointState()
    cmd.position = [b,0,0]
    cmd.name = ['Joint_1','Joint_2','Joint_3']
    delay = 0.2
    time.sleep(0.8)
    if(d=='up'):
        angles = np.linspace(-1.57,start,180)
    else:
        angles = np.linspace(start,1.57,180)

    for i in range(len(angles)):
        cmd.position = [b,angles[i],0]
        joint_control.publish(cmd)
        time.sleep(delay)

def look_rl(joint_control,angles,m=0):
    cmd = JointState()
    cmd.position = [0,m,0]
    cmd.name = ['Joint_1','Joint_2','Joint_3']
    delay = 0.01
    time.sleep(0.8)

    for i in range(len(angles)):
        cmd.position = [angles[i],m,0]
        joint_control.publish(cmd)
        time.sleep(delay)

def look_ud(joint_control,angles,m=0):
    cmd = JointState()
    cmd.position = [m,0,0]
    cmd.name = ['Joint_1','Joint_2','Joint_3']
    delay = 0.01
    time.sleep(0.8)

    for i in range(len(angles)):
        cmd.position = [m,angles[i],0]
        joint_control.publish(cmd)
        time.sleep(delay)




def map_sequence(joint_control):
    m_range = np.linspace(1.5,-1.5,5)
    look_rl(joint_control,np.linspace(0,1.57,180))
    look_rl(joint_control,np.linspace(1.57,-1.57,180))
    look_rl(joint_control,np.linspace(-1.57,0,180))

    look_ud(joint_control,np.linspace(-1.57,0,180))
    look_ud(joint_control,np.linspace(0,1.57,180))

    look_ud(joint_control,np.linspace(1.57,0,180),m=3)
    look_ud(joint_control,np.linspace(0,-1.57,180),m=3)

    look_ud(joint_control,np.linspace(-1.57,0,180),m=4)
    look_ud(joint_control,np.linspace(0,1.57,180),m=4)

    look_ud(joint_control,np.linspace(1.57,0,180),m=1)
    look_ud(joint_control,np.linspace(0,-1.57,180),m=1)

    look_ud(joint_control,np.linspace(-1.57,0,180),m=0)
    look_ud(joint_control,np.linspace(0,1.57,180),m=0)
    
def robot_home(joint_control):
    cmd = JointState()
    cmd.name = ['Joint_1','Joint_2','Joint_3']
    cmd.position = [0,0,0] # Robot Home POSITION
    joint_control.publish(cmd)

def move(joint_control,arr):
    cmd = JointState()
    cmd.name = ['Joint_1','Joint_2','Joint_3']
    cmd.position = arr # Robot Home POSITION
    joint_control.publish(cmd)


if __name__=='__main__':
    rospy.init_node('look_around')

    rate = rospy.Rate(10.0)
    cmd = JointState()
    
    joint_control = rospy.Publisher('/joint_states_ct',JointState,queue_size=10) # Publish Joint States
    cmd.name = ['Joint_1','Joint_2','Joint_3']
    cmd.position = [0,0,0] # Robot Home POSITION
    time.sleep(5)
    joint_control.publish(cmd)
    map_sequence(joint_control)
    robot_home(joint_control)
    # while not rospy.is_shutdown():
        # cmd.data = [90,90,90]
        # servo.publish(cmd)
        # joint_control.publish(cmd)
        # rate.sleep()
    # rospy.spin()