#!/usr/bin/env python
#----------------------------------------------------------------------
# Tasks:
# 1. Checks if right trigger on PS3 pressed
# 2. Open/Closes Baxter's right gripper
# 3. Make sure active window is the terminal (rviz may intercept)
#
# Written BY Stephanie L. Chang
# Last updated 5/16/17
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

# Testing
from pykdl_utils.kdl_kinematics import KDLKinematics
import custom_logging as cl
from urdf_parser_py.urdf import URDF
# Testing end

class HandControl():
    def __init__(self):
        rospy.loginfo("Creating HandControl class")

        # Testing
        # Create KDL model of Baxter
        with cl.suppress_stdout_stderr():    # Eliminates urdf tag warnings
            self.robot = URDF.from_parameter_server()
        self.kin = KDLKinematics(self.robot, "base", "right_hand")
        self.names = self.kin.get_joint_names()
        #Testing end

        # Instatiating objects
        self.kb = kbhit.KBHit()
        self.rh = baxter_interface.Gripper("right")

        rospy.sleep(1.0)
        self.run_state = ON
        self.run_state = OFF
        self.gripper_state = OPEN
        self.gripper_state = CLOSED

        # Timers, subscribers and publishers
        self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.run_cb)
        self.omni_grey_sub = rospy.Subscriber('omni1_button', PhantomButtonEvent, self.omni_grey_cb)

    def run_cb(self,kb_msg):
        # Deadman switch
        if self.kb.kbhit(): # Check if key pressed
            kb_input = self.kb.getch()
            if kb_input == 's':
                bad_key = False
                desired_run_state = ON
            elif kb_input == 'f':
                bad_key = False
                desired_run_state = OFF
            else:
                rospy.loginfo("Invalid key. Please try again.")
                bad_key = True

            if (bad_key == False):
                if self.run_state is not desired_run_state:
                    self.run_state = desired_run_state
        return

    def omni_grey_cb(self,omni_msg):
        # Gripper switch
        if (self.run_state and omni_msg.white_button): # ON and button pressed
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
