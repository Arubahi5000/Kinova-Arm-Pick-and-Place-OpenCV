#!/usr/bin/env python

####################
# test file for master operations 
####################

from __future__ import print_function

import json
import rospy
from math import radians as rad
from arm.srv import ObjectMoveAction, MoveAction, GripperAction

def ObjectMoveActionFunc(jsonDic):
    rospy.wait_for_service('/my_gen3/object_move_action')
    try:
        objectMoveAction = rospy.ServiceProxy('/my_gen3/object_move_action', ObjectMoveAction)
        success = objectMoveAction(jsonDic)
        return success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def MoveActionFunc(jsonDic):
    rospy.wait_for_service('/my_gen3/move_action')
    try:
        moveAction = rospy.ServiceProxy('/my_gen3/move_action', MoveAction)
        success = moveAction(jsonDic)
        return success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def GripperActionFunc(jsonDic):
    rospy.wait_for_service('/my_gen3/gripper_action')
    try:
        gripperAction = rospy.ServiceProxy('/my_gen3/gripper_action', GripperAction)
        success = gripperAction(jsonDic)
        return success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    print("starting")
    #print(GripperActionFunc(json.dumps(1)))
    #print(MoveActionFunc(json.dumps([1,1,1])))
    print(ObjectMoveActionFunc(json.dumps(
        {'move_y':0.5, 'move_x':0.1}
    )))