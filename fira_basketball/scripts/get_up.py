#! /usr/bin/env python

import rospy
from op3_ros_utils import *

rospy.init_node("sprint")

robot = Robot()
# rospy.sleep(3)

def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position
    robot.setGeneralControlModule("action_module")

    #robot.setGrippersPos(left=60.0, right=60.0)

    # Call initial robot position
    robot.playMotion(1, wait_for_end=True)

    # Set ctrl module to walking, this actually only sets the legs
    rospy.sleep(1.0)


init()


robot.setGeneralControlModule("action_module")
robot.playMotion(82, wait_for_end=True)
rospy.sleep(0.25)


rospy.spin()


