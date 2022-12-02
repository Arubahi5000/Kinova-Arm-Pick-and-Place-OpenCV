#!/usr/bin/env python3

####################
# unused 
####################

import rospy
from std_msgs.msg import String, Int32

def callback(data, other):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data + " from " + other)

def depth_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " " + data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter1", String, callback, callback_args="1")
    rospy.Subscriber("depth_value", Int32, depth_callback)
    rospy.spin()

if __name__ == '__main__':
  listener()