#!/usr/bin/env python
#-----------------------------------------------------------------------
# Class version of velcon.py
#
# Task:
# 1. Finds Js and Vs
# 2. Uses naive least-squares damping to find qdot
#   (Addresses inf vel at singularities)
# 3. Publishes qdot as JointCommand msg to Baxter joint_command topic
#
# Last Updated: 4/2/17
#-----------------------------------------------------------------------
# Python Imports
import numpy as np
from math import pow, pi
import tf.transformations as tr

# ROS Imports
import rospy
from std_msgs.msg import Bool, Int32, Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from baxter_core_msgs.msg import JointCommand, EndpointState
import baxter_interface

# Local Imports
from baxter_right_description import right_char
import modern_robotics as r
import custom_logging as cl

DAMPING = 0.005

class VelocityController(object):
    def __init__(self):
        rospy.logwarn("Creating VelocityController class")

        # Create KDL model
        with cl.suppress_stdout_stderr():    # Eliminates urdf tag warnings
            self.robot = URDF.from_parameter_server()
        self.kin = KDLKinematics(self.robot, "base", "right_hand")
        self.current_pose = Pose()
        self.desired_pose = Pose()
        self.names = self.kin.get_joint_names()

        # Shared variables
        self.damping = rospy.get_param("~damping", DAMPING)
        self.current_pose = Pose()
        self.desired_pose = Pose()
        self.qdot_stored = JointCommand()
        self.qdot_stored.mode = 2
        self.qdot_stored.command = np.zeros(7)
        self.qdot_stored.names = self.names
        self.arm = baxter_interface.Limb("right")
        self.drive_right = 0.5
        self.drive_left = -0.5

        # Flags
        self.running_flag = 0
        self.current_pose_flag = 0
        self.desired_pose_flag = 0

        # Subscribers
        rospy.Subscriber('start_move', Bool, self.start_move_cb)
        rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.current_pose_cb)
        rospy.Subscriber('target_position', Pose, self.new_pose_cb)

        # Publishers
        self.qdot_pub = rospy.Publisher('robot/limb/right/joint_command', JointCommand, queue_size = 3)
        self.move_done_pub = rospy.Publisher('move_done', Bool, queue_size = 3)

        while True:
            self.calc_ang_vel()

    def start_move_cb(self, from_cmd):   # Toggling run flag
        if from_cmd:
            self.running_flag = 1        # Enables velocity control
        else:
            self.running_flag = 0        # Disables velocity control

    def current_pose_cb(self, ref):      # Stores reference pose
        if self.running_flag:
            self.current_pose = ref.pose

    def new_pose_cb(self, goal):         # Stores pose published by commander
        if self.running_flag:
            self.desired_pose = goal

    def quat_arr(self, some_pose):   # Reformats quaternion as np.array
        quat = np.array([some_pose.orientation.x, some_pose.orientation.y, some_pose.orientation.z, some_pose.orientation.w])
        return quat

    def p_arr(self, some_pose):      # Reformats position as np.array
        p = np.array([some_pose.position.x, some_pose.position.y, some_pose.position.z])
        return p

    def current_thetalist(self): # Returns joint angles at current config
        i = 0
        thetalist = np.zeros(7)
        while i<7:
            thetalist[i] = self.arm.joint_angle(self.names[i])
            i = i + 1
        return thetalist

    def calc_ang_vel(self):
        if self.running_flag:
            print "========================================="
            rospy.logwarn("Calculating joint velocities.")
            self.running_flag = 0    # Resetting flag

            # Grab M0 and Slist from baxter_right_description.py
            parts = right_char()
            M0 = parts[0] #Zero config of right_hand
            Slist = parts[1] #6x7 screw axes mx of right arm

            # Current config: base to actual - Tba
            p_now = self.p_arr(self.current_pose)
            quat_now = self.quat_arr(self.current_pose)
            th_now = tr.euler_from_quaternion(quat_now, 'sxyz') # Rot ang
            T_now = tr.compose_matrix(angles=th_now, translate=p_now)
            print "p_now", p_now
            print "th_now", th_now
            print "T_now", T_now

            # Desired config: base to desired - Tbd
            p_goal = self.p_arr(self.desired_pose)
            quat_goal = self.quat_arr(self.desired_pose)
            th_goal = tr.euler_from_quaternion(quat_goal, 'sxyz') # Rot ang
            T_goal = tr.compose_matrix(angles=th_goal, translate=p_goal)

            # Construct SPACE JACOBIAN for current config
            q_now = self.current_thetalist() #1x7 mx - grab current joint ang
            print "q_now", q_now
            Js = r.JacobianSpace(Slist, q_now) #6x7 mx

            # Find transform from current pose to desired pose, error
            e = np.dot(r.TransInv(T_now), T_goal)  #e = Tad = TabTbd
            rospy.logwarn("transform from actual to desired: error")
            print e

            # Desired TWIST: MatrixLog6 SE(3) -> se(3) exp coord
            Vs = r.se3ToVec(r.MatrixLog6(e))

            # Desired ang vel - Eq 5 from Chiaverini & Siciliano, 1994
            # Managing singularities: naive least-squares damping
            n = Js.shape[-1] #Size of last row, n = 7 joints
            invterm = np.linalg.inv(np.dot(Js.T, Js) + pow(self.damping, 2)*np.eye(n))
            qdot_new = np.dot(np.dot(invterm,Js.T),Vs)

            rospy.logwarn("qdot calculated")
            print qdot_new

            #Building velocity JointCommand msg
            qdot = JointCommand()
            qdot.mode = 2 # Velocity mode
            qdot.command = qdot_new #1x7
            qdot.names = self.names

            #Check if joint limit reached
            if q_now[0] <= -1.70167993878:
                print "Lower joint 1 limit reached"
                qdot.command[0] = self.drive_right
            elif q_now[0] >= 1.70167993878:
                print "Upper joint 1 limit reached"
                qdot.command[0] = self.drive_left

            if q_now[1] <= -2.147:
                print "Lower joint 2 limit reached"
                qdot.command[1] = self.drive_right
            elif q_now[1] >= 1.047:
                print "Upper joint 2 limit reached"
                qdot.command[1] = self.drive_left

            if q_now[2] <= -3.05417993878:
                print "Lower joint 3 limit reached"
                qdot.command[2] = self.drive_right
            elif q_now[2] >= 3.05417993878:
                print "Upper joint 3 limit reached"
                qdot.command[2] = self.drive_left

            if q_now[3] <= -0.05:
                print "Lower joint 4 limit reached"
                qdot.command[3] = self.drive_right
            elif q_now[3] >= 2.618:
                print "Upper joint 4 limit reached"
                qdot.command[3] = self.drive_left

            if q_now[4] <= -3.059:
                print "Lower joint 5 limit reached"
                qdot.command[4] = self.drive_right
            elif q_now[4] >= 3.059:
                print "Upper joint 5 limit reached"
                qdot.command[4] = self.drive_left

            if q_now[5] <= -1.57079632679:
                print "Lower joint 6 limit reached"
                qdot.command[5] = self.drive_right
            elif q_now[5] >= 2.094:
                print "Upper joint 6 limit reached"
                qdot.command[5] = self.drive_left

            if q_now[6] <= -3.059:
                print "Lower joint 7 limit reached"
                qdot.command[6] = self.drive_right
            elif q_now[6] >= 3.059:
                print "Upper joint 7 limit reached"
                qdot.command[6] = self.drive_left

            rospy.logwarn("qdot.command after limit check")
            print qdot.command

            #Check if arm oscillating
            rospy.logwarn("Max difference between new and old qdot")
            max_dv = 0
            i = 0
            while (i<7):
                dv = abs(qdot.command[i] - self.qdot_stored.command[i])
                if dv > max_dv:
                    max_dv = dv
                i = i+1
            print max_dv

            if max_dv < 0.1:
                qdot.command = [0, 0, 0, 0, 0, 0, 0]
                self.move_done_pub.publish(True)

            self.qdot_pub.publish(qdot) # Publishing velocity
            self.qdot_stored = qdot

        else:
            self.move_done_pub.publish(False)


def main():
    rospy.init_node('velocity_controller')

    try:
        v_control = VelocityController()
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__ == '__main__':
    main()
