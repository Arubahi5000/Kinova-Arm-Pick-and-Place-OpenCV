#!/usr/bin/env python3

####################
# unused 
####################

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

class depth_node():
    def __init__(self) -> None:
        self.depth = 0
        self.pub = rospy.Publisher('depth_value', Int32, queue_size=10)

    def send_depth(self, depth):
        rospy.loginfo("depth: "+depth)
        self.pub.publish(depth)

    def callback(self, image):

        print(f"Image step is {image.step}")
        print(f"Image height is {image.height}")
        print(f"Image width is {image.width}")
        print(image.data[145 * 480])

        #process
        #self.send_depth(result)

    def start(self):
        rospy.init_node('depth_value_chatter', anonymous=True)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.callback)
        #rospy.Subscriber("/camera/depth/points", PointCloud2, self.callback)
        rospy.spin()

if __name__ == '__main__':
    this_node = depth_node()
    this_node.start()
