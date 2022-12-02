#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from math import radians as rad
from std_srvs.srv import Empty

class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        self.gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True


  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(trajectory_message, wait=True)

  def reach_joint_angles(self, tolerance):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      joint_positions[0] = pi/2
      joint_positions[1] = 0
      joint_positions[2] = pi/4
      joint_positions[3] = -pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
      joint_positions[6] = 0.2
    elif self.degrees_of_freedom == 6:
      joint_positions[0] = 0
      joint_positions[1] = 0
      joint_positions[2] = pi/2
      joint_positions[3] = pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 

def main():
  example = ExampleMoveItTrajectories()

  # For testing purposes
  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass
  
  # #   # Send the goal
  # #   success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)

  # # if example.is_gripper_present and success:
  # #   rospy.loginfo("Opening the gripper...")
  # #   success &= example.reach_gripper_position(0)
  # #   print (success)

  # #   rospy.loginfo("Closing the gripper 50%...")
  # #   success &= example.reach_gripper_position(0.5)
  # #   print (success)

  # if success:
  #   rospy.loginfo("Reaching Named Target Home...")
  #   success &= example.reach_named_position("home")
  #   print (success)
  #   example.arm_group.stop()

  object_found = False

  if success and not object_found:
    rospy.loginfo("Going to home to start...")
    success &= example.reach_named_position("home")
    print(success)
    rospy.loginfo("Reached home to start complete")

  if success and example.is_gripper_present and not object_found:
    rospy.loginfo("Opening gripper...")
    success &= example.reach_gripper_position(0)
    print(success)

  home_x = 0.65
  home_y = 0.00
  x = home_x
  y = home_y
  rospy.loginfo("Going to pose top left corner...")
  rospy.loginfo("Starting pose for pose top left corner is: ")
  example.get_cartesian_pose()
  while (x > 0.4):
    x -= 0.05
    y -= 0.05
    if success:
      new_pose = [x, y, 0.2, rad(180), rad(0), rad(90)]
      success &= example.reach_cartesian_pose(pose=new_pose, tolerance=0.01, constraints=None)
      print(success)
  rospy.loginfo("Reached pose top left corner complete")
  # if success:
  #   rospy.loginfo("Moving to pose 1 in segments...")
  #   rospy.loginfo("Current pose is: ")
  #   curr_pose = example.get_cartesian_pose()
  #   curr_pose.orientation.x = rad(180)
  #   success &= example.reach_cartesian_pose(pose=curr_pose, tolerance=0.01, constraints=None)
  #   print(success)

  #   curr_pose = example.get_cartesian_pose()
  #   curr_pose.orientation.y = rad(0)
  #   success &= example.reach_cartesian_pose(pose=curr_pose, tolerance=0.01, constraints=None)
  #   print(success)

  #   curr_pose = example.get_cartesian_pose()
  #   curr_pose.orientation.z = rad(90)
  #   success &= example.reach_cartesian_pose(pose=curr_pose, tolerance=0.01, constraints=None)
  #   print(success)
    
  #   curr_pose = example.get_cartesian_pose()
  #   curr_pose.position.x = 0.4
  #   success &= example.reach_cartesian_pose(pose=curr_pose, tolerance=0.01, constraints=None)
  #   print(success)
    
  #   curr_pose = example.get_cartesian_pose()
  #   curr_pose.position.y = -0.25
  #   success &= example.reach_cartesian_pose(pose=curr_pose, tolerance=0.01, constraints=None)
  #   print(success)
    
  #   curr_pose = example.get_cartesian_pose()
  #   curr_pose.position.z = 0.2
  #   success &= example.reach_cartesian_pose(pose=curr_pose, tolerance=0.01, constraints=None)
  #   print(success)
  # rospy.loginfo("Completed segmented pose 1")

  # if success:
  #   rospy.loginfo("Going to pose top left corner...")
  #   rospy.loginfo("Starting pose for pose top left corner is: ")
  #   example.get_cartesian_pose()
  #   new_pose = [0.4, -0.25, 0.2, rad(180), rad(0), rad(90)]
  #   success &= example.reach_cartesian_pose(pose=new_pose, tolerance=0.01, constraints=None)
  #   print(success)

  if success and not object_found:
    rospy.loginfo("Going to pose middle left...")
    rospy.loginfo("Starting pose for pose middle left is: ")
    example.get_cartesian_pose()
    new_pose = [0.2, -0.25, 0.2, rad(180), rad(0), rad(90)]
    success &= example.reach_cartesian_pose(pose=new_pose, tolerance=0.01, constraints=None)
    print(success)
    rospy.loginfo("Reached pose middle left complete")

  if success and not object_found:
    rospy.loginfo("Going to pose bottom left corner...")
    rospy.loginfo("Starting pose for pose bottom left corner is: ")
    example.get_cartesian_pose()
    new_pose = [0, -0.25, 0.2, rad(180), rad(0), rad(90)]
    success &= example.reach_cartesian_pose(pose=new_pose, tolerance=0.01, constraints=None)
    rospy.loginfo("Ending pose for pose bottom left is: ")
    # example.get_cartesian_pose()
    print(success)
    rospy.loginfo("Reached pose bottom left complete")

  if success and not object_found:
    rospy.loginfo("Going to pose bottom middle...")
    rospy.loginfo("Starting pose for pose bottom middle is: ")
    example.get_cartesian_pose()
    # y = -0.25
    # while (y > -0.375):
    #   y -= 0.025
    #   new_pose = [0, y, 0.2, rad(180), rad(0), rad(90)]
    #   success &= example.reach_cartesian_pose(pose=new_pose, tolerance=0.01, constraints=None)
    new_pose = [0, -0.375, 0.2, rad(180), rad(0), rad(90)]
    success &= example.reach_cartesian_pose(pose=new_pose, tolerance=0.01, constraints=None)
    rospy.loginfo("Ending pose for pose bottom middle is: ")
    # example.get_cartesian_pose()
    print(success)
    rospy.loginfo("Reached pose bottom middle complete")

  if success and not object_found:
    rospy.loginfo("Going to pose middle middle...")
    rospy.loginfo("Starting pose for pose middle middle is: ")
    example.get_cartesian_pose()
    new_pose = [0.2, -0.375, 0.2, rad(180), rad(0), rad(90)]
    success &= example.reach_cartesian_pose(pose=new_pose, tolerance=0.01, constraints=None)
    rospy.loginfo("Ending pose for pose middle middle is: ")
    # example.get_cartesian_pose()
    print(success)
    rospy.loginfo("Reached pose middle middle complete")

  # Imagine see object
  # object_found = True
  # rospy.loginfo("OBJECT FOUND!!!")

  if success and not object_found:
    rospy.loginfo("Going to pose top middle...")
    rospy.loginfo("Starting pose for pose top middle is: ")
    example.get_cartesian_pose()
    new_pose = [0.4, -0.375, 0.2, rad(180), rad(0), rad(90)]
    success &= example.reach_cartesian_pose(pose=new_pose, tolerance=0.01, constraints=None)
    rospy.loginfo("Ending pose for pose top middle is: ")
    # example.get_cartesian_pose()
    print(success)
    rospy.loginfo("Reached pose top middle complete")

  if success and not object_found:
    rospy.loginfo("Going to pose top right corner...")
    rospy.loginfo("Starting pose for pose top right corner is: ")
    example.get_cartesian_pose()
    new_pose = [0.4, -0.5, 0.2, rad(180), rad(0), rad(90)]
    success &= example.reach_cartesian_pose(pose=new_pose, tolerance=0.01, constraints=None)
    rospy.loginfo("Ending pose for pose top right corner is: ")
    # example.get_cartesian_pose()
    print(success)
    rospy.loginfo("Reached pose top right corner complete")

  if success and not object_found:
    rospy.loginfo("Going to pose middle right corner...")
    rospy.loginfo("Starting pose for pose middle right corner is: ")
    example.get_cartesian_pose()
    new_pose = [0.2, -0.5, 0.2, rad(180), rad(0), rad(90)]
    success &= example.reach_cartesian_pose(pose=new_pose, tolerance=0.01, constraints=None)
    rospy.loginfo("Ending pose for pose middle right corner is: ")
    # example.get_cartesian_pose()
    print(success)
    rospy.loginfo("Reached pose middle right corner complete")

  if success and not object_found:
    rospy.loginfo("Going to pose bottom right corner...")
    rospy.loginfo("Starting pose for pose bottom right corner is: ")
    example.get_cartesian_pose()
    new_pose = [0, -0.5, 0.2, rad(180), rad(0), rad(90)]
    success &= example.reach_cartesian_pose(pose=new_pose, tolerance=0.01, constraints=None)
    rospy.loginfo("Ending pose for pose bottom right corner is: ")
    # example.get_cartesian_pose()
    print(success)
    rospy.loginfo("Reached pose bottom right corner complete")

  if success and not object_found:
    rospy.loginfo("Going to top right pose to go home...")
    rospy.loginfo("Starting pose for top right pose to go home is: ")
    example.get_cartesian_pose()
    new_pose = [0.4, -0.5, 0.2, rad(180), rad(0), rad(90)]
    success &= example.reach_cartesian_pose(pose=new_pose, tolerance=0.01, constraints=None)
    print(success)
    rospy.loginfo("Reached top right pose to go home")

  if success and not object_found:
    rospy.loginfo("Going to top left pose to go home...")
    rospy.loginfo("Starting pose for top left pose to go home is: ")
    example.get_cartesian_pose()
    new_pose = [0.4, -0.25, 0.2, rad(180), rad(0), rad(90)]
    success &= example.reach_cartesian_pose(pose=new_pose, tolerance=0.01, constraints=None)
    print(success)
    rospy.loginfo("Reached top left pose to go home")

  if object_found:
    x_offset = 0.03
    y_offset = -0.03
    if success:
      rospy.loginfo("Going towards object...")
      rospy.loginfo("Current pose is: ")
      curr_pose = example.get_cartesian_pose()
      curr_pose.position.x += x_offset
      curr_pose.position.y += y_offset
      success &= example.reach_cartesian_pose(pose=curr_pose, tolerance=0.01, constraints=None)
      print(success)
      rospy.loginfo("Currently above object")

    z_offset = 0.06
    if success:
      rospy.loginfo("Picking up object...")
      rospy.loginfo("Current pose before picking up object: ")
      curr_pose = example.get_cartesian_pose()
      curr_pose.position.z -= z_offset
      drop_off_z = curr_pose.position.z
      success &= example.reach_cartesian_pose(pose=curr_pose, tolerance=0.01, constraints=None)
      print(success)
      rospy.loginfo("Currently ready to grip the object")

    if success and example.is_gripper_present:
      rospy.loginfo("Gripping the object...")
      success &= example.reach_gripper_position(0.50)
      print(success)
      rospy.loginfo("Currently gripped object")

    if success:
      rospy.loginfo("Lifting up object...")
      rospy.loginfo("Current pose before lifting up object: ")
      curr_pose = example.get_cartesian_pose()
      curr_pose.position.z = 0.3
      success &= example.reach_cartesian_pose(pose=curr_pose, tolerance=0.01, constraints=None)
      print(success)
      rospy.loginfo("Lifted up object")

    if success:
      rospy.loginfo("Going towards drop off location...")
      curr_joints = example.arm_group.get_current_joint_values()
      curr_joints[0] -= pi/2
      example.arm_group.set_joint_value_target(curr_joints)
      success &= example.arm_group.go(wait=True)
      print(success)
      rospy.loginfo("Current pose above drop off location")

    if success:
      rospy.loginfo("Dropping off object...")
      rospy.loginfo("Current pose before dropping off object: ")
      curr_pose = example.get_cartesian_pose()
      curr_pose.position.z = drop_off_z
      success &= example.reach_cartesian_pose(pose=curr_pose, tolerance=0.01, constraints=None)
      print(success)
      rospy.loginfo("Current pose ready to let go off object")

    if success and example.is_gripper_present:
      rospy.loginfo("Letting go off object...")
      success &= example.reach_gripper_position(0)
      print(success)
      rospy.loginfo("Let go of object")

    if success:
      rospy.loginfo("Lifting up from object...")
      rospy.loginfo("Current pose before lifting up from object: ")
      curr_pose = example.get_cartesian_pose()
      curr_pose.position.z = 0.2
      success &= example.reach_cartesian_pose(pose=curr_pose, tolerance=0.01, constraints=None)
      print(success)
      rospy.loginfo("Lifted up from object")

  # Always return to home to finish
  if success:
    rospy.loginfo("Going to home to end...")
    success &= example.reach_named_position("home")
    print (success)
  rospy.loginfo("Reached home to end complete")

  # For testing purposes
  rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

  if not success:
      rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  main()
