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
# Last Updated: 3/30/17
#-----------------------------------------------------------------------
# Python Imports
import numpy as np
from math import pow

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

        # Shared variables
        self.damping = rospy.get_param("~damping", DAMPING)
        self.current_pose = Pose()
        self.desired_pose = Pose()
        self.qdot_stored = JointCommand()
        self.qdot_stored.mode = 2
        self.qdot_stored.command = [0, 0, 0, 0, 0, 0, 0]
        self.qdot_stored.names = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]

        # Flags
        self.running_flag = 0
        self.current_pose_flag = 0
        self.desired_pose_flag = 0

        # Create KDL model
        #self.robot = URDF.from_xml_file("/home/stephanie/Projects/catkin_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")

        with cl.suppress_stdout_stderr():    # Eliminates urdf tag warnings
            self.robot = URDF.from_parameter_server()
        self.kin = KDLKinematics(self.robot, "base", "right_hand")
        self.current_pose = Pose()
        self.desired_pose = Pose()

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
            #rospy.logwarn("Inside start_move_cb")
            self.running_flag = 1        # Enables velocity control
        else:
            self.running_flag = 0        # Disables velocity control

    def current_pose_cb(self, ref):      # Stores reference pose
        if self.running_flag:
            self.current_pose = ref.pose

    def new_pose_cb(self, goal):         # Stores pose published by commander
        if self.running_flag:
            self.desired_pose = goal

    def calc_ang_vel(self):
        if self.running_flag:
            rospy.logwarn("Calculating joint velocities.")
            self.running_flag = 0    # Resetting flag

            # Grab M0 and Slist from baxter_right_description.py
            parts = right_char()

            M0 = parts[0] #Zero config of right_hand
            Slist = parts[1] #6x7 screw axes mx of right arm

            rospy.logwarn("before q_now")

            # Find joint angles and transform mx of current config
            #with cl.suppress_stdout_stderr():
            q_now = self.kin.inverse(self.current_pose) #1x7 matrix
            while (q_now is None):    # Keep trying until soln' found
                q_now = self.kin.inverse(self.current_pose)
            T_now = r.FKinSpace(M0, Slist, q_now)

            rospy.logwarn("before q_goal")

            # Find transform mx to DESIRED_POSE
            q_goal = self.kin.inverse(self.desired_pose)
            while (q_goal is None):    # Keep trying until soln' found
                rospy.logwarn("Searching for solution...")
                q_goal = self.kin.inverse(self.desired_pose)
            T_goal = r.FKinSpace(M0, Slist, q_goal)

            print q_goal

            # Construct SPACE JACOBIAN for current config
            Js = r.JacobianSpace(Slist, q_now) #6x7 matrx

            # Find transform from current pose to desired pose, error
            e = np.dot(T_now, T_goal)   #e = TnowTgoal

            # Desired TWIST: MatrixLog6 SE(3) -> se(3) exp coord
            Vs = r.se3ToVec(r.MatrixLog6(e))

            # Desired ang vel - Eq 5 from Chiaverini & Siciliano, 1994
            # Managing singularities: naive least-squares damping
            n = Js.shape[-1] #Size of last row, n = 7 joints
            invterm = np.linalg.inv(np.dot(Js.T, Js) + pow(self.damping, 2)*np.eye(n))
            qdot_new = np.dot(np.dot(invterm,Js.T),Vs)

            #Building JointCommand msg for publishing
            qdot = JointCommand()
            qdot.mode = 2 # Velocity mode
            qdot.command = [qdot_new[0], qdot_new[1], qdot_new[2], qdot_new[3], qdot_new[4], qdot_new[5], qdot_new[6]]
            qdot.names = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]

            #Find maximum difference between new and old velocities
            rospy.logwarn("Finding max difference between new and old qdot")
            max_diff = 0
            i = 0
            while (i<7):
                diff = abs(qdot.command[i] - self.qdot_stored.command[i])
                if diff > max_diff:
                    max_diff = diff
                    print max_diff
                i = i+1

            print max_diff

            if max_diff < 0.5:
                qdot.command = [0, 0, 0, 0, 0, 0, 0]

            #     # Position control
            #     q = JointCommand()
            #     q.mode = 1 # Position mode
            #     q.command = [q_goal[0], q_goal[1], q_goal[2], q_goal[3], q_goal[4], q_goal[5], q_goal[6]]
            #     q.names = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]
            #     self.qdot_pub.publish(q)
            #     self.move_done_pub.publish(True) # Target reached
            # else:
            self.qdot_pub.publish(qdot) # Publishing velocity
            self.qdot_stored = qdot
            self.move_done_pub.publish(False)

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
