#!/usr/bin/env python

import sys
import rospy
import math
from time import sleep

from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes

class ObjectFollowerNode:
    def __init__(self):
        self.timer = 0
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.sub_boundingBoxes, queue_size=10)
        self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        rospy.Timer(rospy.Duration(0.01), self.update_follower)

    def sub_boundingBoxes(self, bb_msg):
        for box in bb_msg.bounding_boxes:
            rospy.loginfo(
                "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}".format(
                box.xmin, box.xmax, box.ymin, box.ymax
            )
        )

    def update_follower(self, event):
        self.timer+=1

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    try :
        rospy.init_node('object_follower_node')
        node = ObjectFollowerNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
