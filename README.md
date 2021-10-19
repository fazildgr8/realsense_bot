# realsense_bot
This is a ROS package for Intel realsense D435i with 3-DOF Manipulator robot that can be used for Indoor Mapping and localization of objects in the world frame with an added advantage of the robot's dexterity. The 3-DOF Manipulator is a self-built custom robot where the URDF with the depth sensor is included. The package covers the Rosserial based nodes to control the robot's Joint States and PCL pipelines required for autonomous mapping/Localization/Tracking of the objects in real-time. <br/>

## Robot URDF Model
![ROBOT_Description](https://user-images.githubusercontent.com/24454678/138001080-06d19e65-3412-4b5f-99dd-e5860527b5dc.png)

## Manual Joint Control with PCL Perception
Launch the Robot in RVIZ with Manual Joint Control
```
roslaunch realsense_bot robot_visualize_control.launch
```

Launch the IntelRealsense Camera node to start the PCL Perception
```
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud align_depth:=true depth_width:=640 depth_height:=480 depth_fps:=30 color_width:=640 color_height:=480 color_fps:=30
```
![manual_perception](https://user-images.githubusercontent.com/24454678/138002158-53b967c4-bc66-44af-9593-714cdea28551.gif)

## Autonomous Object Tracking using YoloV3 and 2D to 3D deprojection with Pose Estimation 
Launch the Yolo detector node to produce Object Detection and Bounding Boxes Messages
```
roslaunch yolo_detect.launch
```
Launch the node which deprojects the 2D detected object in 3D using rs2_pixel_to_point_deprojection(...)
```
rosrun realsense_bot yolo_pixel_to_3Dpoint.py
```
Launch the node which follows and tracks the Object using the 3-DOF Manipulator
```
rosrun realsense_bot object_follow.py
```
![object_following](https://user-images.githubusercontent.com/24454678/138001224-60c70811-e5c0-4361-8236-de49b074e4ec.gif)
## 3-DOF Dexterity based RTAB Mapping with Madwick IMU Filter
Launch the below mentioned seperately to begin RTAB-Mapping with Madwick IMU Filter and RVIZ representation of the Cloud Map.
```
roslaunch realsense_bot robot_visualize_control.launch
roslaunch realsense_bot realsense_mapping.launch
```
![rtab_map_room](https://user-images.githubusercontent.com/24454678/138001579-007b174b-27c4-4620-b38a-7e0a500d18b2.gif)
