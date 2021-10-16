#!/usr/bin/python
import roslib
import rospy
import tf
import math
import random
from std_msgs.msg import Int16MultiArray
import numpy as np
import time


if __name__=='__main__':
    rospy.init_node('servo_control')

    rate = rospy.Rate(10.0)
    cmd = Int16MultiArray()
    servo = rospy.Publisher('/servo', Int16MultiArray,queue_size=10) # Publish Command
    cmd.data = [90,90,90] # Robot Home POSITION
    servo.publish(cmd)
    while not rospy.is_shutdown():
        # cmd.data = [90,90,90]
        # servo.publish(cmd)
        rate.sleep()
