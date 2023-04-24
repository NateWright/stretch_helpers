#!/usr/bin/env python3

import sys
import rospy
from actionlib_msgs.msg import GoalStatusArray

def callback(msg: GoalStatusArray):
    if len(msg.status_list) and msg.status_list[-1].status == 1:
        print("Moving")
    else:
        print("Not Moving")

# print("initializing")
rospy.init_node('listener', anonymous=True)

rospy.Subscriber("/move_base/status", GoalStatusArray, callback)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()