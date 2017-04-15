#!/usr/bin/env python
#----------------------------------------------------------------------
# Tasks:
# 1. Checks if right trigger on PS3 pressed
# 2. Open/Closes right gripper
#
# Last updated 4/13/17
#----------------------------------------------------------------------
import rospy
from sensor_msgs.msg import Joy
import intera_interface

####################
# GLOBAL VARIABLES #
####################
DEADMAN_INDEX = 10
GRIPPER_INDEX = 11
OPEN = True
CLOSED = False

class HandControl():
    def __init__(self):
        rospy.loginfo("Creating HandControl class")

        self.rh = intera_interface.gripper.Gripper("right")
        rospy.sleep(1.0)
        self.gripper_state = OPEN
        self.gripper_state = CLOSED

        # Subscriber
        self.right_trigger_sub = rospy.Subscriber('joy', Joy, self.right_trigger_cb, queue_size=1)

    def right_trigger_cb(self,joy_msg):
        if (joy_msg.buttons[GRIPPER_INDEX] and joy_msg.buttons[DEADMAN_INDEX]):
            desired_state = CLOSED
            command = self.rh.close
        else:
            desired_state = OPEN
            command = self.rh.open
        # print "desired = ",desired_state
        # print "internal = ",self.gripper_state
        if self.gripper_state is not desired_state:
            rospy.loginfo("Changing gripper state to %s", desired_state)
            command()
            self.gripper_state = desired_state
        return

def main():
    rospy.init_node('gripper_open_close')

    try:
        grab = HandControl()
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__ == '__main__':
    main()
