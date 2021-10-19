# realsense_bot
This is a ROS package for Intel realsense D435i with 3-DOF Manipulator robot that can be used for Indoor Mapping and localization of objects in the world frame with an added advantage of the robot's dexterity. The 3-DOF Manipulator is a self-built custom robot where the URDF with the depth sensor is included. The package covers the Rosserial based nodes to control the robot's Joint States and PCL pipelines required for autonomous mapping/Localization/Tracking of the objects in real-time. <br/>

## Robot URDF Model
![ROBOT_Description](https://user-images.githubusercontent.com/24454678/138001080-06d19e65-3412-4b5f-99dd-e5860527b5dc.png)

## Manual Joint Control with PCL Perception
'roslaunch realsense_bot robot_visualize_control.launch'
![manual_perception](https://user-images.githubusercontent.com/24454678/138002158-53b967c4-bc66-44af-9593-714cdea28551.gif)

## Autonomous Object Tracking and 3D Pose Estimation
![object_following](https://user-images.githubusercontent.com/24454678/138001224-60c70811-e5c0-4361-8236-de49b074e4ec.gif)

## 3-DOF Dexterity based RTAB Mapping with Madwick IMU Filter
![rtab_map_room](https://user-images.githubusercontent.com/24454678/138001579-007b174b-27c4-4620-b38a-7e0a500d18b2.gif)
