#!/usr/bin/env python
#----------------------------------------------------------------------
# Tasks:
# 1. Checks if right trigger on PS3 pressed
# 2. Open/Closes Baxter's right gripper
#
# Written BY Stephanie L. Chang
# Last updated 5/16/17
#----------------------------------------------------------------------
import rospy
from phantom_omni.msg import PhantomButtonEvent
import baxter_interface

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

        self.rh = baxter_interface.Gripper("right")
        rospy.sleep(1.0)
        self.run_state = ON
        self.run_state = OFF
        self.gripper_state = OPEN
        self.gripper_state = CLOSED

        # Subscribers and publishers
        self.omni_grey_sub = rospy.Subscriber('omni1_button', PhantomButtonEvent, self.omni_grey_cb)
        self.key_sub = rospy.Subscriber('', , self.run_cb)

    def run_cb(self,key_msg):
        # Deadman switch: 's' for on, 'f' for off
        if (key_msg):
            
        else if (key_msg):

        if self.run_state is not desired_run_state:
            self.run_state = desired_run_state
        return

    def omni_grey_cb(self,omni_msg):
        # Gripper switch
        if (omni_msg.white_button): # ON and pressed
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
