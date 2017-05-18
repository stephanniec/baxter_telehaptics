#!/usr/bin/env python
#----------------------------------------------------------------------
# Tasks:
# 1. Checks if white button on Phantom Omni is pressed
# 2. Open/Closes Baxter's right gripper
# 3. Make sure active window is the terminal (rviz may intercept)
#
# Written BY Stephanie L. Chang
# Last updated 5/17/17
#----------------------------------------------------------------------
# ROS Imports
import rospy

# Local Imports
import baxter_interface
import kbhit
from phantom_omni.msg import PhantomButtonEvent

####################
# GLOBAL VARIABLES #
####################
OPEN = True
CLOSED = False
ON = True
OFF = False

class HandControl():
    def __init__(self):
        rospy.loginfo("Creating HandControl class")

        # Instatiating objects
        self.kb = kbhit.KBHit()
        rospy.on_shutdown(self.kb.set_normal_term)
        self.rh = baxter_interface.Gripper("right")

        rospy.sleep(1.0)
        self.run_state = ON # Always on
        self.gripper_state = OPEN
        self.gripper_state = CLOSED

        # Subscribers
        self.omni_grey_sub = rospy.Subscriber('omni1_button', PhantomButtonEvent, self.omni_white_cb, queue_size=1)
        return

    def run_cb(self,kb_msg):
        # Deadman switch
        if self.kb.kbhit(): # Check if key pressed
            kb_input = self.kb.getch()
            if kb_input == 's':
                desired_run_state = ON
                self.run_state = ON
                rospy.loginfo("Set run_state to ON.")
            elif kb_input == 'f':
                desired_run_state = OFF
                self.run_state = OFF
                rospy.loginfo("Set run_state to OFF.")
            else:
                rospy.loginfo("Invalid key. Please try again.")
        return

    def omni_white_cb(self,omni_msg):
        # Gripper switch
        if self.run_state:
            if omni_msg.white_button: # ON and button pressed
                desired_state = CLOSED
                command = self.rh.close
            else:
                desired_state = OPEN
                command = self.rh.open

            if self.gripper_state is not desired_state:
                rospy.loginfo("changing gripper state to %s", desired_state)
                command()
                self.gripper_state = desired_state
        return

def main():
    rospy.init_node('omni_gripper_open_close')
    rospy.loginfo('Inside omni gripper node')

    try:
        grab = HandControl()
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__ == '__main__':
    main()
