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
# Last Updated: 4/3/17
#-----------------------------------------------------------------------
# Python Imports
import numpy as np
from math import pow, pi
import tf.transformations as tr
import threading

# ROS Imports
import rospy
from std_msgs.msg import Bool, Int32, Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from baxter_core_msgs.msg import JointCommand, EndpointState
import baxter_interface

# Local Imports
from baxter_right_description import right_char
import modern_robotics as r
import custom_logging as cl

DAMPING = 0.005
JOINT_VEL_LIMIT = 2

class VelocityController(object):
    def __init__(self):
        rospy.logwarn("Creating VelocityController class")

        # Create KDL model
        with cl.suppress_stdout_stderr():    # Eliminates urdf tag warnings
            self.robot = URDF.from_parameter_server()
        self.kin = KDLKinematics(self.robot, "base", "right_hand")
        self.names = self.kin.get_joint_names()

        # Shared variables
        self.damping = rospy.get_param("~damping", DAMPING)
        self.joint_vel_limit = rospy.get_param("~joint_vel_limit", JOINT_VEL_LIMIT)
        self.mutex = threading.Lock()
        self.arm = baxter_interface.Limb("right")
        self.q = np.zeros(7) # Joint angles
        self.T_goal = np.array(self.kin.forward(self.q)) # Ref se3
        self.qdot = JointCommand()
        self.qdot.mode = 2 # Velocity mode
        self.qdot.names = self.names
        self.qdot_stored = JointCommand()
        self.qdot_stored.mode = 2
        self.qdot_stored.command = np.zeros(7)
        self.qdot_stored.names = self.names

        self.drive_right = -1
        self.drive_left = 1

        # Flags
        self.running_flag = 0

        # Subscribers
        self.start_move_cb_sub = rospy.Subscriber('start_move', Bool, self.start_move_cb)
        self.current_js_cb_sub = rospy.Subscriber('/robot/joint_states', JointState, self.current_js_cb)
        self.ref_se3_cb_sub = rospy.Subscriber('target_position', Pose, self.ref_se3_cb)

        # Publishers
        self.joint_cmd_pub = rospy.Publisher('robot/limb/right/joint_command', JointCommand, queue_size = 3)
        self.move_done_pub = rospy.Publisher('move_done', Bool, queue_size = 3)

        while True:
            self.calc_ang_vel()

    def start_move_cb(self, from_cmd):   # Toggling run flag
        if from_cmd:
            self.running_flag = 1        # Enables velocity control
        else:
            self.running_flag = 0        # Disables velocity control

    def current_js_cb(self, js):         # Grabs current joint angles
        qtemp = np.zeros(7)
        names = self.names
        i = 0
        for j,n in enumerate(js.name):
            if n in names:
                qtemp[i] = js.position[j]
                i += 1
        with self.mutex:
            self.q = qtemp
        return

    def ref_se3_cb(self, some_pose):   # Takes target pose, returns desired se3
        with self.mutex:
            p = np.array([some_pose.position.x, some_pose.position.y, some_pose.position.z])
            quat = np.array([some_pose.orientation.x, some_pose.orientation.y, some_pose.orientation.z, some_pose.orientation.w])
            self.T_goal = tr.compose_matrix(angles=tr.euler_from_quaternion(quat, 'sxyz'), translate=p)
        return

    def check_joint_limits(self):      #Check if joint limit reached
        rospy.logwarn("Checking joint limits...")
        if self.q[0] <= -1.5:
            print "Lower joint 1 limit reached"
            self.qdot.command[0] = self.drive_right
        elif self.q[0] >= 1.5:
            print "Upper joint 1 limit reached"
            self.qdot.command[0] = self.drive_left

        if self.q[1] <= -2.0:
            print "Lower joint 2 limit reached"
            self.qdot.command[1] = self.drive_right
        elif self.q[1] >= 1.0:
            print "Upper joint 2 limit reached"
            self.qdot.command[1] = self.drive_left

        if self.q[2] <= -3.0:
            print "Lower joint 3 limit reached"
            self.qdot.command[2] = self.drive_right
        elif self.q[2] >= 3.0:
            print "Upper joint 3 limit reached"
            self.qdot.command[2] = self.drive_left

        if self.q[3] <= -0.05:
            print "Lower joint 4 limit reached"
            self.qdot.command[3] = self.drive_right
        elif self.q[3] >= 2.5:
            print "Upper joint 4 limit reached"
            self.qdot.command[3] = self.drive_left

        if self.q[4] <= -3.0:
            print "Lower joint 5 limit reached"
            self.qdot.command[4] = self.drive_right
        elif self.q[4] >= 3.0:
            print "Upper joint 5 limit reached"
            self.qdot.command[4] = self.drive_left

        if self.q[5] <= -1.5:
            print "Lower joint 6 limit reached"
            self.qdot.command[5] = self.drive_right
        elif self.q[5] >= 2.0:
            print "Upper joint 6 limit reached"
            self.qdot.command[5] = self.drive_left

        if self.q[6] <= -3.0:
            print "Lower joint 7 limit reached"
            self.qdot.command[6] = self.drive_right
        elif self.q[6] >= 3.0:
            print "Upper joint 7 limit reached"
            self.qdot.command[6] = self.drive_left

    def stop_oscillations(self):     #Check if arm oscillating
            rospy.logwarn("Max difference between new and old qdot")
            max_dv = 0
            i = 0
            while (i<7):
                dv = abs(self.qdot.command[i] - self.qdot_stored.command[i])
                if dv > max_dv:
                    max_dv = dv
                i = i+1
            print max_dv

            if max_dv < 0.5:
                self.qdot.command = [0, 0, 0, 0, 0, 0, 0]
                self.move_done_pub.publish(True)

    def calc_ang_vel(self):
        if self.running_flag:
            print "========================================="
            rospy.logwarn("Calculating joint velocities.")
            self.running_flag = 0    # Resetting flag

            # Grab M0 and Slist from baxter_right_description.py
            parts = right_char()
            M0 = parts[0] #Zero config of right_hand
            Slist = parts[1] #6x7 screw axes mx of right arm

            #Current joint angles
            q_now = self.q #1x7 mx
            print "q_now", q_now

            # Desired config: base to desired - Tbd
            with self.mutex:
                T_bd = self.T_goal

            # Find transform from current pose to desired pose, error
            e = np.dot(r.TransInv(r.FKinSpace(M0, Slist, q_now)), T_bd)  #e = Tad = TabTbd
            print "error", e

            # Desired TWIST: MatrixLog6 SE(3) -> se(3) exp coord
            Vs = r.se3ToVec(r.MatrixLog6(e))

            # Construct SPACE JACOBIAN for current config
            Js = r.JacobianSpace(Slist, q_now) #6x7 mx

            # Desired ang vel - Eq 5 from Chiaverini & Siciliano, 1994
            # Managing singularities: naive least-squares damping
            n = Js.shape[-1] #Size of last row, n = 7 joints
            invterm = np.linalg.inv(np.dot(Js.T, Js) + pow(self.damping, 2)*np.eye(n))
            qdot_new = np.dot(np.dot(invterm,Js.T),Vs)
            print "raw qdot calculated", qdot_new

            # Scaling joint velocity
            minus_v = abs(np.amin(qdot_new))
            plus_v = abs(np.amax(qdot_new))
            if minus_v > plus_v:
                scale = minus_v
            else:
                scale = plus_v
            if scale > self.joint_vel_limit:
                qdot_new = (qdot_new/scale)*self.joint_vel_limit

            self.qdot.command = qdot_new #1x7
            print "scaled qdot", self.qdot.command

            #Safety checks - will modify self.qdot.command if needed
            self.check_joint_limits()
            self.stop_oscillations()

            #self.joint_cmd_pub.publish(self.qdot) # Publishing velocity
            self.qdot_stored.command = self.qdot.command
            print "qdot_stored.command", self.qdot_stored.command

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
