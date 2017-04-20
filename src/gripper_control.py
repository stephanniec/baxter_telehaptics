#!/usr/bin/env python
#----------------------------------------------------------------------
# Tasks:
# 1. Checks if right trigger on PS3 pressed
# 2. Open/Closes Baxter's right gripper
#
# Written BY Stephanie L. Chang
# Last updated 4/11/17
#----------------------------------------------------------------------
import rospy
from sensor_msgs.msg import Joy
import baxter_interface

####################
# GLOBAL VARIABLES #
####################
DEADMAN_INDEX = 10
GRIPPER_INDEX = 11

class HandControl():
    def __init__(self):
        rospy.loginfo("Creating HandControl class")

        self.rh = baxter_interface.Gripper("right")

        # Subscriber
        self.right_trigger_sub = rospy.Subscriber('joy', Joy, self.right_trigger_cb)

    def right_trigger_cb(self,joy_msg):
        if (joy_msg.buttons[GRIPPER_INDEX] and joy_msg.buttons[DEADMAN_INDEX]):
            self.rh.close()
        else:
            self.rh.open()
        return

def main():
    rospy.init_node('gripper_open_close')

    try:
        grab = HandControl()
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__ == '__main__':
    main()
