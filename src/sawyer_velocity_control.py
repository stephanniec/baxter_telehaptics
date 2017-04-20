#!/usr/bin/env python
#-----------------------------------------------------------------------
# Stripped down version of velocity_controller.py
# Runs at 100Hz
#
# Tasks:
# 1. Finds Jb and Vb
# 2. Uses naive least-squares damping to find qdot
#   (Addresses inf vel at singularities)
# 3. Publishes qdot to Sawyer using the limb interface
#
# Written By Stephanie L. Chang
# Last Updated: 4/13/17
#-----------------------------------------------------------------------
# Python Imports
import numpy as np
from math import pow, pi, sqrt
import tf.transformations as tr
import threading

# ROS Imports
import rospy
from std_msgs.msg import Bool, Int32, Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import intera_interface

# Local Imports
import sawyer_MR_description as s
import modern_robotics as r
import custom_logging as cl

####################
# GLOBAL VARIABLES #
####################
TIME_LIMIT = 15    #15s
DAMPING = 0.03
JOINT_VEL_LIMIT = 2    #2rad/s

class VelocityControl(object):
    def __init__(self):
        rospy.loginfo("Creating VelocityController class")

        # Create KDL model
        with cl.suppress_stdout_stderr():    # Eliminates urdf tag warnings
            self.robot = URDF.from_parameter_server()
        self.kin = KDLKinematics(self.robot, "base", "right_hand")
        self.names = self.kin.get_joint_names()

        # Limb interface
        self.arm = intera_interface.Limb("right")
        self.hand = intera_interface.gripper.Gripper('right')

        # Grab M0 and Blist from saywer_MR_description.py
        self.M0 = s.M #Zero config of right_hand
        self.Blist = s.Blist #6x7 screw axes mx of right arm

        # Shared variables
        self.mutex = threading.Lock()
        self.time_limit = rospy.get_param("~time_limit", TIME_LIMIT)
        self.damping = rospy.get_param("~damping", DAMPING)
        self.joint_vel_limit = rospy.get_param("~joint_vel_limit", JOINT_VEL_LIMIT)
        self.q = np.zeros(7)        # Joint angles
        self.qdot = np.zeros(7)     # Joint velocities
        self.T_goal = np.array(self.kin.forward(self.q))    # Ref se3

        # Subscriber
        self.ref_pose_sub = rospy.Subscriber('ref_pose', Pose, self.ref_pose_cb)

        self.hand.calibrate()

        self.r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.calc_joint_vel()
            self.r.sleep()

    def ref_pose_cb(self, some_pose): # Takes target pose, returns ref se3
        rospy.logdebug("ref_pose_cb called in velocity_control.py")
        p = np.array([some_pose.position.x, some_pose.position.y, some_pose.position.z])
        quat = np.array([some_pose.orientation.x, some_pose.orientation.y, some_pose.orientation.z, some_pose.orientation.w])
        goal_tmp = tr.compose_matrix(angles=tr.euler_from_quaternion(quat, 'sxyz'), translate=p)
        with self.mutex:
            self.T_goal = goal_tmp

    def get_q_now(self):         # Finds current joint angles
        qtemp = np.zeros(7)
        i = 0
        while i<7:
            qtemp[i] = self.arm.joint_angle(self.names[i])
            i += 1
        with self.mutex:
            self.q = qtemp              # Angles in radians

    def stop_oscillating(self):
        i = 0
        v_norm = 0
        qvel = self.qdot

        while i<7:
            v_norm += pow(qvel[i],2)
            i += 1
        v_norm = sqrt(v_norm)

        if v_norm < 0.1:
            self.qdot = np.zeros(7)
        return

    def calc_joint_vel(self):
        rospy.logdebug("Calculating joint velocities...")

        # Body stuff
        Tbs = self.M0
        Blist = self.Blist

        # Current joint angles
        self.get_q_now()
        with self.mutex:
            q_now = self.q #1x7 mx

        # Desired config: base to desired - Tbd
        with self.mutex:
            T_sd = self.T_goal

        # Find transform from current pose to desired pose, error
        e = np.dot(r.TransInv(r.FKinBody(Tbs, Blist, q_now)), T_sd)

        # Desired TWIST: MatrixLog6 SE(3) -> se(3) exp coord
        Vb = r.se3ToVec(r.MatrixLog6(e))
        # Construct BODY JACOBIAN for current config
        Jb = r.JacobianBody(Blist, q_now) #6x7 mx

        # Desired ang vel - Eq 5 from Chiaverini & Siciliano, 1994
        # Managing singularities: naive least-squares damping
        n = Jb.shape[-1] #Size of last row, n = 7 joints
        invterm = np.linalg.inv(np.dot(Jb.T, Jb) + pow(self.damping, 2)*np.eye(n))
        qdot_new = np.dot(np.dot(invterm,Jb.T),Vb)

        # Scaling joint velocity
        minus_v = abs(np.amin(qdot_new))
        plus_v = abs(np.amax(qdot_new))
        if minus_v > plus_v:
            scale = minus_v
        else:
            scale = plus_v
        if scale > self.joint_vel_limit:
            qdot_new = 1.0*(qdot_new/scale)*self.joint_vel_limit
        self.qdot = qdot_new #1x7

        # Constructing dictionary
        qdot_output = dict(zip(self.names, self.qdot))

        # Setting Sawyer right arm joint velocities
        self.arm.set_joint_velocities(qdot_output)
        return

def main():
    rospy.init_node('velocity_control')

    try:
        vc = VelocityControl()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

if __name__ == '__main__':
    main()
