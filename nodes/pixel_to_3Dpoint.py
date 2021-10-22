#!/usr/bin/python
import roslib
import rospy
import tf
from sensor_msgs.msg import Image,CameraInfo
import numpy as np
from rospy.numpy_msg import numpy_msg
import time
import pyrealsense2
from cv_bridge import CvBridge, CvBridgeError
import cv2


def convert_depth_to_phys_coord_using_realsense(x, y, depth, cameraInfo):  
    _intrinsics = pyrealsense2.intrinsics()
    _intrinsics.width = cameraInfo.width
    _intrinsics.height = cameraInfo.height
    _intrinsics.ppx = cameraInfo.K[2]
    _intrinsics.ppy = cameraInfo.K[5]
    _intrinsics.fx = cameraInfo.K[0]
    _intrinsics.fy = cameraInfo.K[4]
    #_intrinsics.model = cameraInfo.distortion_model
    _intrinsics.model  = pyrealsense2.distortion.none
    _intrinsics.coeffs = [i for i in cameraInfo.D]  
    result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)  
    #result[0]: right, result[1]: down, result[2]: forward
    return result


if __name__=='__main__':
    rospy.init_node('pixel_to_3Dpoint')
    rospy.Subscriber("/camera/color/camera_info", CameraInfo, cam_info) 
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        pose_tf = np.array(pose)
        br.sendTransform((pose_tf[2]/1000, -pose_tf[0]/1000, pose_tf[1]/1000),
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "object",
                            "camera_link")
    # rate.sleep()
    rospy.spin()