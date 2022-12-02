#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time
from kortex_driver.srv import *
from kortex_driver.msg import *

class ExampleCartesianActionsWithNotifications:
    def __init__(self):
        try:
            rospy.init_node('example_cartesian_poses_with_notifications_python')

            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            self.all_notifs_succeeded = True

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")

            rospy.loginfo("Using robot_name " + self.robot_name)

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                time.sleep(0.01)

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            # rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                rospy.loginfo("Waiting for Action")
                return self.wait_for_action_end_or_abort()
    
    def send_to_pose(self, req, pose_num):
        rospy.loginfo(f"Sending pose {pose_num}...")
        self.last_action_notif_type = None
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr(f"Failed to send pose {pose_num}")
            return False
        else:
            rospy.loginfo(f"Waiting for pose {pose_num} to finish...")
        return self.wait_for_action_end_or_abort()
        rospy.sleep(1)


    def main(self):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python")
        except:
            pass

        if success:

            #*******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            try:
                self.example_clear_faults()
            except:
                rospy.logerr("Couldn't clear faults")
            #*******************************************************************************
            
            #*******************************************************************************
            # Start the example from the Home position
            try:
                self.example_home_the_robot()
            except:
                rospy.logerr("Couldn't Home Robot")
            #*******************************************************************************
            
            # Prepare and send pose 1
            my_cartesian_speed = CartesianSpeed()
            my_cartesian_speed.translation = 0.1 # m/s
            my_cartesian_speed.orientation = 15  # deg/s

            my_constrained_pose = ConstrainedPose()
            my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

            my_constrained_pose.target_pose.x = 0.4
            my_constrained_pose.target_pose.y = -0.25
            my_constrained_pose.target_pose.z = 0.2
            my_constrained_pose.target_pose.theta_x = 180
            my_constrained_pose.target_pose.theta_y = 0
            my_constrained_pose.target_pose.theta_z = 90

            req = ExecuteActionRequest()
            req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
            req.input.name = "pose1"
            req.input.handle.action_type = ActionType.REACH_POSE
            req.input.handle.identifier = 1001

            self.send_to_pose(req, 1)

            # Prepare and send pose 2
            req.input.handle.identifier = 1002
            req.input.name = "pose2"

            my_constrained_pose.target_pose.x = 0

            req.input.oneof_action_parameters.reach_pose[0] = my_constrained_pose

            self.send_to_pose(req, 2)

            # Prepare and send pose 3
            req.input.handle.identifier = 1003
            req.input.name = "pose3"

            my_constrained_pose.target_pose.y = -0.5

            req.input.oneof_action_parameters.reach_pose[0] = my_constrained_pose

            self.send_to_pose(req, 3)

            # Prepare and send pose 4
            req.input.handle.identifier = 1003
            req.input.name = "pose4"

            my_constrained_pose.target_pose.x = 0.4

            req.input.oneof_action_parameters.reach_pose[0] = my_constrained_pose

            self.send_to_pose(req, 4)

            # success &= self.all_notifs_succeeded

            #success &= self.all_notifs_succeeded

            # Home the robot to end
            #success &= self.example_home_the_robot()

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = ExampleCartesianActionsWithNotifications()
    ex.main()
