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
# Last Updated: 3/20/17
#-----------------------------------------------------------------------
import rospy
import numpy as np
from math import pow
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from baxter_right_description import right_char
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import modern_robotics as r
from baxter_core_msgs.msg import JointCommand, EndpointState
import baxter_interface

DAMPING = 0.005

class VelocityController(object):
    def __init__(self):
        rospy.logwarn("Creating VelocityController class")

        # Shared variables
        self.damping = rospy.get_param("~damping", DAMPING)
        self.current_pose = Pose()
        self.desired_pose = Pose()

        # Create KDL model
        #self.robot = URDF.from_xml_file("/home/stephanie/Projects/catkin_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")
        self.robot = URDF.from_parameter_server()
        self.kin = KDLKinematics(self.robot, "base", "right_hand")
        self.current_pose = Pose()
        self.desired_pose = Pose()
        #self.limb_interface = baxter_interface.Limb()

        # Flags
        self.running_flag = False
        self.current_pose_flag = 0
        self.desired_pose_flag = 0

        # Subscribers
        rospy.Subscriber('start_move', Bool, self.start_move_cb)
        rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.current_pose_cb)
        rospy.Subscriber('target_position', Pose, self.new_pose_cb)

        # Publishers
        qdot_pub = rospy.Publisher('robot/limb/right/joint_command', JointCommand, queue_size = 10)
        move_done_pub = rospy.Publisher('move_done', Bool, queue_size = 10)

    def start_move_cb(self, from_cmd):    # Toggling run flag
        if from_cmd:
            rospy.logwarn("Inside start_move_cb")
            self.running_flag = 1        # Start velocity control
        else:
            rospy.logwarn("else start_move_cb")
            self.running_flag = 0

    def current_pose_cb(self, ref):    # Stores reference pose
        if self.running_flag:
            rospy.logwarn("Inside current_pose_cb")
            self.running_flag = 0    # Reset flag
            self.current_pose = ref.pose

    def new_pose_cb(self, goal):    # Stores pose published by commander
        if self.running_flag:
            rospy.logwarn("Inside new_pose_cb")
            self.desired_flag = 0    # Reset flag
            self.desired_pose = goal

    def calc_ang_vel(self):
        if self.running_flag:
            rospy.logwarn("Inside calc_ang_vel")

            # Grab M0 and Slist from baxter_right_description.py
            parts = right_char()
            M0 = parts[0] #Zero config of right_hand
            Slist = parts[1] #6x7 screw axes mx of right arm

            # Find joint angles and transform mx of current config
            q_now = self.kin.inverse(self.current_pose) #1x7 matrix

            rospy.logwarn("q_now")
            print q_now

            T_now = r.FKinSpace(M0, Slist, q_now)

            # Find transform mx to DESIRED_POSE
            q_goal = self.kin.inverse(self.desired_pose)

            rospy.logwarn("q_goal")
            print q_goal

            T_goal = r.FKinSpace(M0, Slist, q_goal)

            # Construct SPACE JACOBIAN for current config
            Js = r.JacobianSpace(Slist, q_now) #6x7 matrx

            # Find transform from current pose to desired pose, error
            e = np.dot(T_now, T_goal)   #e = TnowTgoal

            # Desired TWIST
            Vs = r.se3ToVec(r.MatrixLog6(e)) #MatrixLog6 SE(3) -> se(3) exp coord

            # Desired ang vel - Eq 5 from Chiaverini & Siciliano, 1994
            # Managing singularities: naive least-squares damping
            n = Js.shape[-1] #Size of last row, n = 7 joints
            invterm = np.linalg.inv(np.dot(Js.T, Js) + pow(self.damping, 2)*np.eye(n))
            qdot = np.dot(np.dot(invterm,Js.T),Vs)

            #Preparing to send new angular velocities

            #limb = baxter_interface.Limb('right')   # Instance of limb class
            #qdot_names = self.kin.get_joint_names()

            #Building JointCommand msg for publishing
            qdot_new = JointCommand()
            qdot_new.mode = 2 #Velocity mode
            qdot_new.command = [qdot[0], qdot[1], qdot[2], qdot[3], qdot[4], qdot[5], qdot[6]]
            qdot_new.names = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]
            self.qdot_pub.publish(qdot_new) #default is 5Hz
            rospy.logwarn("Sending new pose")

            self.move_done_pub.publish(True)

def main():
    rospy.init_node('velocity_controller')

    try:
        v_control = VelocityController()
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__ == '__main__':
    main()
