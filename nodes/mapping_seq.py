#!/usr/bin/python
import roslib
import rospy
import tf
import math
import random
from std_msgs.msg import Int16MultiArray
import numpy as np
import time


def look_around_ccw(servo,m,u=90):
    cmd = Int16MultiArray()
    cmd.data = [0,m,u]
    delay = 0.1
    time.sleep(0.8)
    servo.publish(cmd)

    for i in range(0,180):
        cmd.data = [i,m,u]
        servo.publish(cmd)
        time.sleep(delay)


def look_around_cw(servo,m,u=90):
    cmd = Int16MultiArray()
    cmd.data = [0,m,u]
    delay = 0.1
    time.sleep(0.8)
    servo.publish(cmd)

    for i in range(0,180):
        cmd.data = [180-i,m,u]
        servo.publish(cmd)
        time.sleep(delay)

def look_around_m(servo):
    m_range = np.linspace(25,125,10)
    u = 90
    for i in range(len(m_range)):
        # if m_range[i]<100:
        if i%2 == 0:
            look_around_ccw(servo,m_range[i])
        else:
            look_around_cw(servo,m_range[i])
        # else:
        #     if i%2 == 0:
        #         look_around_ccw(servo,m_range[i],m_range[i])
        #     else:
        #         look_around_cw(servo,m_range[i],180-m_range[i])


def robot_home(servo):
    cmd = Int16MultiArray()
    cmd.data = [90,90,90] # Robot Home POSITION
    servo.publish(cmd)


if __name__=='__main__':
    rospy.init_node('look_around')

    rate = rospy.Rate(10.0)
    cmd = Int16MultiArray()
    servo = rospy.Publisher('/servo', Int16MultiArray,queue_size=10) # Publish Command
    cmd.data = [90,90,90] # Robot Home POSITION
    servo.publish(cmd)
    # look_around(servo,90)
    look_around_m(servo)
    robot_home(servo)
    # while not rospy.is_shutdown():
    #     # cmd.data = [90,90,90]
    #     # servo.publish(cmd)
    #     rate.sleep()
