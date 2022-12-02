roslaunch kortex_gazebo spawn_kortex_robot.launch gripper:=robotiq_2f_140 start_rviz:=false

roslaunch kortex_driver kortex_driver.launch gripper:=robotiq_2f_140

roslaunch kinova_vision kinova_vision.launch

rosrun image_view image_view image:=/camera/color/image_raw