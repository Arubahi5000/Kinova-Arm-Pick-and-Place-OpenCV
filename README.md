# Introduction

This was part of our COMP3431 Robotic Architecture Final Project at UNSW Computer Science Engineering. We had to conduct a pick and place using the OpenCV Computer Vision library. The KINOVA arm was running on ROS KORTEX and using the MoveIt and KORTEX Vision Packages. Unfortunately, due to time constraints were were not able to complete the task fully however we had the individual components to create a successful project if we only we had a bit more time.

# Follow these steps to replicate our demo

1. roslaunch kortex_gazebo spawn_kortex_robot.launch gripper:=robotiq_2f_140 start_rviz:=false

2. roslaunch kortex_driver kortex_driver.launch gripper:=robotiq_2f_140

3. roslaunch kinova_vision kinova_vision.launch

4. rosrun image_view image_view image:=/camera/color/image_raw