#!/usr/bin/env python
#----------------------------------------------------------------------
#
# Tasks:
# 1. Reads OMNI tf output
# 2. Maps controller positions to a target pose using velocities
# 3. Defines EE pose work space
#
# Written by Stephanie L. Chang
# Last updated 5/17/17
#----------------------------------------------------------------------
# ROS Imports
import rospy
import tf
import tf.transformations as tr
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

#Python Imports
import numpy as np

#Local Imports
import kbhit

def calc_quat(th):
    return tr.quaternion_from_euler(3.115649698300095, -0.02399721058403635, th, 'rxyz')

####################
# GLOBAL VARIABLES #
####################
ZERO_POS = np.array([0.7, -0.45, 0.0]) #Baxter start pos
ZERO_ORI = calc_quat(0.0)
XYZ_MIN = np.array([0.4, -0.75, -0.25]) #Baxter abs limits
XYZ_MAX = np.array([1.0, -0.25, 0.25])  #Baxter abs limits
XYZ_SCALE = np.array([1.0, 1.25, 1.25])
XYZ_INVERT_MAP = np.array([-1, -1, -1])
XYZ_INDEX_ARR = np.array([0, 1, 2]) #With np.choose: 0th element, first val, etc.
XYZ_OFFSET_ARR = np.array([0.206, 0.0, 0.047]) #For position from /omni/base to /omni/stylus
TH_SCALE = 2.5
TH_MIN = -np.pi/2
TH_MAX = np.pi/2
TH_OFFSET = np.pi/2
FREQUENCY = 20
REF_FRAME = "base"
TARGET_FRAME = "target"
OMNI_BASE_FRAME = "omni/base"
STYLUS_FRAME = "omni/stylus"
ON = True
OFF = False

class SimpleTargets(object):
    def __init__(self):
        rospy.loginfo("Creating SimpleTargets class")

        # setup local variables:
        self.position = ZERO_POS
        self.orientation = ZERO_ORI
        self.ee_dot = np.array([0,0,0])
        self.run_flag = ON
        self.run_flag = OFF

        # Instantiating keyboard object
        self.key = kbhit.KBHit()
        rospy.on_shutdown(self.key.set_normal_term)

        #Subscribers and Publishers
        self.br = tf.TransformBroadcaster()
        self.listen = tf.TransformListener()
        self.key_timer = rospy.Timer(rospy.Duration(0.01), self.running_cb)
        self.js_sub = rospy.Subscriber('/omni1_joint_states', JointState, self.js_cb)
        self.ref_pose_pub = rospy.Publisher('ref_pose', Pose, queue_size = 3)
        self.run_status_pub = rospy.Publisher('/run_status', Bool, queue_size = 3)

        return

    def running_cb(self, key_msg):
        # Deadman switch
        if self.key.kbhit(): # Check if key pressed
            key_input = self.key.getch()
            if key_input == 's':
                bad_key = False
                desired_sys_state = ON
                rospy.loginfo("Pressed 's'")
            elif key_input == 'f':
                bad_key = False
                desired_sys_state = OFF
                rospy.loginfo("Pressed 'f'")
            else:
                bad_key = True
                rospy.loginfo("Unknown key")

            if (bad_key == False): # Checking for valid key
                if self.run_flag is not desired_sys_state:
                    self.run_flag = desired_sys_state
                    rospy.loginfo("Running: %s", self.run_flag)

            self.run_status_pub.publish(desired_sys_state)

    def js_cb(self, omni_js_msg):
        # Enable movement if 's' pressed
        if (self.run_flag):
            try:
                omni_p, omni_quat = self.listen.lookupTransform(OMNI_BASE_FRAME, STYLUS_FRAME, rospy.Time())
            except(tf.Exception):
                print "tf exception encountered"
                return

            # omni stylus tf from omni/base
            p_list =  np.array(omni_p) - XYZ_OFFSET_ARR
            w3_angle = omni_js_msg.position[5]

            # set target position:
            new_pos = XYZ_INVERT_MAP*XYZ_SCALE*np.choose(XYZ_INDEX_ARR, p_list)
            # make sure nominal pose is within limits:
            new_pos_clip = np.clip(ZERO_POS-new_pos, XYZ_MIN, XYZ_MAX)

            # set target angle:
            new_ang = -TH_SCALE*w3_angle
            # capping rotation
            new_ang_clip = np.clip(new_ang, TH_MIN, TH_MAX)

            # update position and orientation:
            self.position = new_pos_clip
            self.orientation = calc_quat(new_ang_clip)

        else: # Reset to zero configuration
            self.position = ZERO_POS
            self.orientation = ZERO_ORI

        # publish Pose and send tf
        self.ref_pose_pub.publish(Pose(Point(*self.position), Quaternion(*self.orientation)))
        self.br.sendTransform(self.position, self.orientation, rospy.Time.now(), TARGET_FRAME, REF_FRAME)
        return

def main():
    rospy.init_node('omni_reference_pose_generator')

    try:
        new_pose = SimpleTargets()
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__ == '__main__':
    main()
