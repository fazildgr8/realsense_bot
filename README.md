# realsense_bot
This is a ROS package for Intel realsense D435i with 3-DOF Manipulator robot that can be used for Indoor Mapping and localization of objects in the world frame with an added advantage of the robot's dexterity. The 3-DOF Manipulator is a self-built custom robot where the URDF with the depth sensor is included. The package covers the Rosserial communication with Arduino nodes to control the robot's Joint States and PCL pipelines required for autonomous mapping/Localization/Tracking of the objects in real-time. <br/>

## Robot URDF and Actual Model
The Robot's URDF model represents the exact simplified version of the actual model with measured offsets from joint to joint. </br>
The RRR Robot is built using standard servo 3 x Clamps,1 x Long U Mount and 45deg U Mount. 
<img src="https://user-images.githubusercontent.com/24454678/138737126-afb4df6c-f03a-4bc6-a14d-1eecad08203b.PNG" width="800">

## Jetson Nano Standalone Implementation
```
cd catkin_ws/src
git clone https://github.com/fazildgr8/realsense_bot -b jetson
```
## PC Arduino Implementation
```
cd catkin_ws/src
git clone https://github.com/fazildgr8/realsense_bot -b master
```

## Manual Joint Control with PCL Perception
**Launch the Robot in RVIZ with Manual Joint Control.** </br> This Launch file starts the Robot state publisher which updates the Transformation tree using the Joint states from Joint state publisher. The Robot can be Controlled manually by publishing the Joint states message to the Joint State Controller. 
</br>

For **Jetson Nano** Implementation execute the below mentioned
```
roslaunch realsense_bot robot_visualize_control_jetson.launch
```
</br>
For **PC+Arduino** Implementation execute the below mentioned 
```
roslaunch realsense_bot robot_visualize_control.launch
```

**Launch the IntelRealsense Camera node to start the PCL Perception** </br> This Launch file starts the IntelRealsense camera node which publishes all the messages w.r.t to the Realsense Camera.
```
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud align_depth:=true depth_width:=640 depth_height:=480 depth_fps:=30 color_width:=640 color_height:=480 color_fps:=30
```
<img src="https://user-images.githubusercontent.com/24454678/138002158-53b967c4-bc66-44af-9593-714cdea28551.gif" width="1000">

## Autonomous Object Tracking using YoloV3 and 2D to 3D deprojection with Pose Estimation 
Launch the Yolo detector node to produce Object Detection and Bounding Boxes Messages
```
roslaunch yolo_detect.launch
```
Launch the node which deprojects the 2D detected object in 3D using rs2_pixel_to_point_deprojection(...) with the Node **/yolo_pixel_to_3Dpoint**
![pose_estimation_graph](https://user-images.githubusercontent.com/24454678/138003646-bfd8a96f-2faa-4301-a390-e48fd1e22078.png)
![yolo_depth](https://user-images.githubusercontent.com/24454678/138730293-4cc3b186-c700-404c-8ac8-941fee765d76.PNG)
![3D_pose_estimation_node](https://user-images.githubusercontent.com/24454678/138730310-3efdbd04-1c2e-407d-a012-e316cdfee15a.PNG)
```
rosrun realsense_bot yolo_pixel_to_3Dpoint.py
```

Launch the node which follows and tracks the Object using the 3-DOF Manipulator
```
rosrun realsense_bot object_follow.py
```
<img src="https://user-images.githubusercontent.com/24454678/138001224-60c70811-e5c0-4361-8236-de49b074e4ec.gif" width="1000">


## 3-DOF Dexterity based RTAB Mapping with Madwick IMU Filter
Launch the below mentioned seperately to begin RTAB-Mapping with Madwick IMU Filter and RVIZ representation of the Cloud Map.
```
roslaunch realsense_bot robot_visualize_control.launch
roslaunch realsense_bot realsense_mapping.launch
```
<img src="https://user-images.githubusercontent.com/24454678/138001579-007b174b-27c4-4620-b38a-7e0a500d18b2.gif" width="1000">
</br>
<img src="https://user-images.githubusercontent.com/24454678/138003662-a99dd86d-7da0-41cf-b990-0128b07e2461.png" width="1000">

## Robot Setup
- **Direct Implementation** - The setup for PC with Arduino control uses **serial communication** to control the servos through the Arduino which is capable producing PWM signals.
- **Jetson Nano Standalone Implementation** - The setup and installation differs for the Jeston boards for using it standalone as GPIO pins present would not be able to produce PWM signals for the servo motors instead we will be using a Servo Controller board which uses **I2C** protocol to control the Servos. Added to it RTAB Mapping is currently unstable with Realsense D435i in Jetson Nano due to its instability in publishing PCL data.


### Setup Info (PC + Arduino Control)
#### ROS/Python Library Prerequisites
```
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
sudo apt-get install ros-$ROS_DISTRO-realsense2-description
sudo apt install ros-$ROS_DISTRO-image-transport-plugins
sudo apt install ros-$ROS_DISTRO-rtabmap-ros
sudo apt install ros-$ROS_DISTRO-rosserial
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
pip install pyrealsense2
pip install opencv-python
cd catkin_ws/src
git clone https://github.com/ccny-ros-pkg/imu_tools
git clone https://github.com/leggedrobotics/darknet_ros
cd ..
catkin_make
```
#### Arduino
The **Arduino** code file required for flashing can be found in `arduino/ros_servo_3dof.ino` <br/>
Connect the **three Servo's** signal pin the to `9 10 11` pins of the Arduino. The pin configuration can be modified within `ros_servo_3dof.ino`.
</br>
<img src="https://user-images.githubusercontent.com/24454678/138738121-74b71943-e564-4657-9281-a8921cb3625a.png" width="450">

#### Jetson Nano
<img src="https://user-images.githubusercontent.com/24454678/138787452-f5b73b7e-4b6d-442b-ac4b-dda14786f8d3.PNG" width="450">

