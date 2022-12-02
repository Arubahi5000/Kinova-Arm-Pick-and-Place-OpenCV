#!/usr/bin/env python3
# Assignment: COMP3431 Assignment 2

# Written By: Aryan Bahinipati, Michael Mortlock-Chapman, 
# Jake Tyler and Dayyaan Naeem Kola

from array import array
from time import sleep
import rospy
from std_msgs.msg import String
from threading import Thread
from math import radians as rad
from arm.srv import ObjectMoveAction, MoveAction, GripperAction
import json

class master:
    def __init__(self):
        self.object_found = False
        self.th = 0
        self.ojbect_jsonDic = ""

    #updates current pose of robot
    def position_callback(self, data):
        self.current_pose = json.loads(data.data)
        #rospy.loginfo(json.loads(data.data))

    ####################
    #unused 
    def colour_callback(self, data):
        pass
        #rospy.loginfo(data.data)
    ####################

    #listens to current pose topic and updates this
    def listener(self):
        #rospy.Subscriber("chatter1", String, self.colour_callback)
        rospy.Subscriber("arm_cartesian_pose", String, self.position_callback)
        rospy.spin()

    #for the colour node so it can tell arm to move over object
    #   moves the arm by given x and y
    #   when the colour node has requested to move 5 times, func stops listening to requests
    def handle_ObjectMoveAction(self, req):
        rospy.loginfo("got oma request")
        rospy.loginfo(req.jsonDic)
        self.object_found = True

        if (self.th < 5):
            #creates a new json array to communicate to movement node so it can move
            self.current_pose["position"]["x"] += float(json.loads(req.jsonDic)["move_x"])
            self.current_pose["position"]["y"] += float(json.loads(req.jsonDic)["move_y"])

            self.ojbect_jsonDic = json.dumps([
                self.current_pose["position"]["x"],
                self.current_pose["position"]["y"],
                0.2,
                rad(180), rad(0), rad(90)
            ])
            

            rospy.loginfo("going to object")
            self.MoveActionRequest(self.ojbect_jsonDic)
            self.th += 1
            return 1
        return 0

    #starts a service that allows the colour node to move to object
    def objectMoveAction(self):
        rospy.loginfo("oma server start")
        s = rospy.Service('object_move_action', ObjectMoveAction, self.handle_ObjectMoveAction)
        rospy.spin()

    #wrapper for arm movement request 
    def MoveActionRequest(self, jsonDic):
        rospy.wait_for_service('/my_gen3/move_action')
        try:
            moveAction = rospy.ServiceProxy('/my_gen3/move_action', MoveAction)
            success = moveAction(jsonDic)
            return success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            exit(1)

    #wrapper for gripper movement request 
    def GripperActionFunc(self, jsonDic):
        rospy.wait_for_service('/my_gen3/gripper_action')
        try:
            gripperAction = rospy.ServiceProxy('/my_gen3/gripper_action', GripperAction)
            success = gripperAction(jsonDic)
            return success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    #main fuction with p&p steps
    def main(self):
        scanningPoints = [
            [0.4, -0.25, 0.2, rad(180), rad(0), rad(90)],
            [0.2, -0.25, 0.2, rad(180), rad(0), rad(90)],
            [0, -0.25, 0.2, rad(180), rad(0), rad(90)],
            
            [0, -0.375, 0.2, rad(180), rad(0), rad(90)],
            [0.2, -0.375, 0.2, rad(180), rad(0), rad(90)],
            [0.4, -0.375, 0.2, rad(180), rad(0), rad(90)],

            [0.4, -0.5, 0.2, rad(180), rad(0), rad(90)],
            [0.2, -0.5, 0.2, rad(180), rad(0), rad(90)],
            [0, -0.5, 0.2, rad(180), rad(0), rad(90)],

            [0.4, -0.5, 0.2, rad(180), rad(0), rad(90)],
            [0.4, -0.25, 0.2, rad(180), rad(0), rad(90)]
        ]
        rospy.loginfo("scanning")
        x = 0
        #do scanning movement over search area while the object is not found 
        #   and there is still points to scan over
        while not self.object_found and x < len(scanningPoints):
            
            success = self.MoveActionRequest(json.dumps(scanningPoints[x]))
            if success == 0:
                rospy.loginfo("scanning problem")
                exit(1)
            x+=1

        #object has been found, wait until color movement requests are done
        while self.th < 5:
            sleep(1)

        #pick up object and place at final destination
        #go down from current pose
        self.ojbect_jsonDic
        object_pose = [i for i in json.loads(self.ojbect_jsonDic)]
        object_pose_arr = [object_pose[0], object_pose[1], 0.06, object_pose[3], object_pose[4], object_pose[5]]
        rospy.loginfo(object_pose_arr)
        success = self.MoveActionRequest(json.dumps(object_pose_arr))
        if success == 0:
            rospy.loginfo("pick up down action error")
            exit(1)
        #close gripper
        success = self.GripperActionFunc(json.dumps(0.8))
        if success == 0:
            rospy.loginfo("gripper action error")
            exit(1)
        #go up
        object_pose_arr = [object_pose[0], object_pose[1], 0.3, object_pose[3], object_pose[4], object_pose[5]]
        success = self.MoveActionRequest(json.dumps(object_pose_arr))
        if success == 0:
            rospy.loginfo("pick up up action error")
            exit(1)
        #go to end point
        object_pose_arr = [0.2, 0.375, 0.2, rad(180), rad(0), rad(90)]
        success = self.MoveActionRequest(json.dumps(object_pose_arr))
        if success == 0:
            rospy.loginfo("pick up up action error")
            exit(1)
        #go down
        object_pose_arr = [0.2, 0.375, 0.06, rad(180), rad(0), rad(90)]
        success = self.MoveActionRequest(json.dumps(object_pose_arr))
        if success == 0:
            rospy.loginfo("pick up up action error")
            exit(1)
        #open gripper
        success = self.GripperActionFunc(json.dumps(0))
        if success == 0:
            rospy.loginfo("gripper action error")
            exit(1)
        #go up
        else:
            rospy.loginfo("couldn't find object")

        success = self.MoveActionRequest("")
        if success == 0:
            rospy.loginfo("gripper action error")
            exit(1)
        

    def start(self):
        #open node
        rospy.loginfo("master start")
        rospy.init_node('listener', anonymous=True)
        #set and activate threads 
        listenerThread = Thread(target=self.listener)
        objectMoveActionThread = Thread(target=self.objectMoveAction)
        mainThread = Thread(target=self.main)

        objectMoveActionThread.start()
        listenerThread.start()
        mainThread.start()

#Start program
if __name__ == '__main__':
    main = master()
    main.start()