#!/usr/bin/env python3
# Assignment: COMP3431 Assignment 2

# Written By: Aryan Bahinipati, Michael Mortlock-Chapman, 
# Jake Tyler and Dayyaan Naeem Kola

from time import sleep
import rospy
from std_msgs.msg import String
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
import json
from arm.srv import ObjectMoveAction, MoveAction, GripperAction

# Publishes a string message alerting that the robot is active
def talker():
    pub = rospy.Publisher('chatter1', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

# Publishes a json dictionary of the offest to move to the movement node when 
# it finds the object (i.e. when this function is invoked)
def ObjectMoveActionFunc(jsonDic):
    rospy.wait_for_service('/my_gen3/object_move_action')
    try:
        objectMoveAction = rospy.ServiceProxy('/my_gen3/object_move_action', ObjectMoveAction)
        success = objectMoveAction(jsonDic)
        return success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Helper function that gets an offest in pixels and converts it
# to meters that the robot has to move in the x and y direction
# to move towards the object
def get_offset(x, y):
  m_to_pixel = 0.0003

  offset_x = x - 640
  offset_y = y - 360

  move_x = (offset_x * m_to_pixel) + 0.03
  move_y = (offset_y * m_to_pixel) - 0.065

  return json.dumps({'move_x': move_x, 'move_y': move_y})

# Lower and upper HSV bounds for colours robot is meant to detect
lower = { 'blue':([100, 190, 70]), 'yellow':([12 , 78, 189]),'green':([69 , 95, 139]), 'red':([0 , 140, 197]), 'purple':([111 , 102, 163]), 'orange':([0 , 90, 183])}
upper = { 'blue':([130,250,250]), 'yellow':([54 , 149, 255]),'green':([86 , 255, 255]), 'red':([255 , 178, 220]), 'purple':([133 , 133, 211]), 'orange':([13 , 146, 210])}

# Font colour that will be used on camera screen to display a particular colour.
colors = {'red':(0,0,255), 'green':(0,255,0), 'blue':(255,0,0), 'yellow':(0, 255, 217), 'orange':(0,140,255), 'purple':(211,0,148)}

# Shape and colour object to find
color_to_find = 'blue'
shape_to_find = 'circle'

class image_converter:
  # Sets up a publisher to the image_topic_2
  # Subscribers to camera to get raw image feed
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

  def callback(self,data):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Raw image captured
    frame = cv_image
    frame2 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Applies blurred filter to raw image
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mlist=[]
    clist=[]
    ks=[]

    # Looks for groups of contours in image and stores them in a list
    for (key, value) in upper.items():
        kernel = np.ones((2,2),np.uint8)
        mask = cv2.inRange(hsv, np.array(lower[key]), np.array(upper[key]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.dilate(mask, kernel, iterations=1)
        mlist.append(mask)
        cnts,_ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(cnts)>=1:
            clist.append(cnts[-1])
            ks.append(key)
    
    # Goes through all clusters of contours in the contour list
    for i,cnt in enumerate(clist):
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            cv2.drawContours(frame, [approx], 0, (0), 2)
            x = approx.ravel()[0]
            y = approx.ravel()[1]
        
            # If the shape has three corners it is a triangle, displays the following information to the output camera display
            if len(approx) == 3:
                cv2.putText(frame, ks[i] + " Triangle at " + f"x:{str(x)}, y:{str(y)}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[ks[i]],2)

                # If the shape and colour matches with the object that it is looking for it moves to the objects position
                if shape_to_find == 'triangle' and color_to_find == ks[i]:
                  ObjectMoveActionFunc(get_offset(x, y))
            # Else if the shape has four corners it is either a square or rectangle
            elif len(approx) == 4:
                x2 = approx.ravel()[2]
                y2 = approx.ravel()[3]
                x4 = approx.ravel()[6]
                y4 = approx.ravel()[7]
                side1 = abs(x4-x2)
                side2 = abs(y4-y2)
                
                # If one side is longer then the other it is then a rectangle, displays the following information to the output camera display 
                if side1 * 1.12 <= side2 or side2 * 1.12 <= side1:
                    cv2.putText(frame, ks[i] + " Rectangle at " + f"x:{str(x)}, y:{str(y)}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[ks[i]],2)

                    # If the shape and colour matches with the object that it is looking for it moves to the objects position
                    if shape_to_find == 'rectangle' and color_to_find == ks[i]:
                      ObjectMoveActionFunc(get_offset(x, y))

                # Else it is then a sqaure, displays the following information to the output camera display 
                else:
                    cv2.putText(frame, ks[i] + " Square at " + f"x:{str(x)}, y:{str(y)}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[ks[i]],2)

                    # If the shape and colour matches with the object that it is looking for it moves to the objects position
                    if shape_to_find == 'square' and color_to_find == ks[i]:
                      ObjectMoveActionFunc(get_offset(x, y))
            # Else if the shape has between seven to ten corners it is an arch, displays the following information to the output camera display 
            elif 7 <= len(approx) <= 10:
                cv2.putText(frame, ks[i] + " Arch at " + f"x:{str(x)}, y:{str(y)}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[ks[i]],2)

                # If the shape and colour matches with the object that it is looking for it moves to the objects position
                if shape_to_find == 'arch' and color_to_find == ks[i]:
                  ObjectMoveActionFunc(get_offset(x, y))
            # Else if the shape has more than ten corners it is a circle, displays the following information to the output camera display
            elif len(approx) > 10:
                cv2.putText(frame, ks[i] + " Circle at " + f"x:{str(x)}, y:{str(y)}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[ks[i]],2)
                
                # If the shape and colour matches with the object that it is looking for it moves to the objects position
                if shape_to_find == 'circle' and color_to_find == ks[i]:
                  ObjectMoveActionFunc(get_offset(x, y))
	
    # Image output display
    #print('showing images')
    cv2.imshow("Image window", mask)
    cv2.imshow('frame', cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def empty(img): pass

# Main program that creates an image_converter class that does image recognition
def main(args):
    print('Runnng')

    ic = image_converter()

    print('class run')
    rospy.init_node('image_converter', anonymous=True)

    print('rospy run')
    try:
        print('Runnng')
        rospy.spin()
        print('Runingg spin')
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# Robot sleeps for 12 seconds so it doesn't find an objects before it arrives
# at the search space
if __name__ == '__main__':
    sleep(12)
    main(sys.argv)