#!/usr/bin/python
import roslib
import rospy
import tf
import math
import random
from std_msgs.msg import Int16MultiArray
import numpy as np
import time
from sensor_msgs.msg import JointState
global joint_poses, servo

joint_poses = [90,90,90]



def callback(msg):
    global joint_poses, servo
    cmd = Int16MultiArray()
    joint_1 = msg.position[0]
    joint_2 = msg.position[1]
    joint_3 = msg.position[2]
    joint_poses = [math.degrees(joint_1)+90,math.degrees(-joint_2)+90,math.degrees(joint_3)+90]
    cmd.data = joint_poses
    servo.publish(cmd)
    # print(msg.name)

if __name__=='__main__':
    rospy.init_node('JointState_Servo')

    rate = rospy.Rate(10.0)
    cmd = Int16MultiArray()
    servo = rospy.Publisher('/servo', Int16MultiArray,queue_size=10) # Publish Command
    rospy.Subscriber("joint_states", JointState, callback) # Publish Joint States to Rosserial
    rospy.spin()
    # cmd.data = [90,90,90] # Robot Home POSITION
    # servo.publish(cmd)
    # while not rospy.is_shutdown():
    #     cmd.data = joint_poses
    #     servo.publish(cmd)
    #     # rospy.spin()
    #     rate.sleep()
