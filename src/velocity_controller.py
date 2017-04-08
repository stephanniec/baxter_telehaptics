#!/usr/bin/env python
#-----------------------------------------------------------------------
# Class version of velcon.py
#
# Task:
# 1. Finds Jb and Vb
# 2. Uses naive least-squares damping to find qdot
#   (Addresses inf vel at singularities)
# 3. Publishes qdot as JointCommand msg to Baxter joint_command topic
#
# Last Updated: 4/4/17
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

        self.drive_right = -0.5
        self.drive_left = 0.5

        # Flags
        self.running_flag = 0

        # Subscribers
        self.start_move_cb_sub = rospy.Subscriber('start_move', Bool, self.start_move_cb)
        self.current_js_cb_sub = rospy.Subscriber('/robot/joint_states', JointState, self.current_js_cb)
        self.ref_se3_cb_sub = rospy.Subscriber('target_position', Pose, self.ref_se3_cb)

        # Publishers
        self.joint_cmd_pub = rospy.Publisher('robot/limb/right/joint_command', JointCommand, queue_size = 3)
        self.move_done_pub = rospy.Publisher('move_done', Bool, queue_size = 3)

        while not rospy.is_shutdown:
            self.calc_ang_vel()

    def start_move_cb(self, from_cmd):   # Toggling run flag
        if from_cmd:
            self.running_flag = 1        # Enables velocity control
        else:
            self.running_flag = 0        # Disables velocity control

    def current_js_cb(self, js):         # Returns current joint angles
        qtemp = np.zeros(7)
        names = self.names
        i = 0
        for j,n in enumerate(js.name):
            if n in names:
                qtemp[i] = js.position[j]
                i += 1
        with self.mutex:
            self.q = qtemp              # Angles in radians

    def ref_se3_cb(self, some_pose):   # Takes target pose, returns desired se3
        p = np.array([some_pose.position.x, some_pose.position.y, some_pose.position.z])
        quat = np.array([some_pose.orientation.x, some_pose.orientation.y, some_pose.orientation.z, some_pose.orientation.w])
        goal_tmp = tr.compose_matrix(angles=tr.euler_from_quaternion(quat, 'sxyz'), translate=p)
        with self.mutex:
            self.T_goal = goal_tmp

    def check_joint_limits(self):      #Check if joint limit reached
        rospy.logwarn("Checking joint limits...")
        if self.q[0] <= -1.5:
            print "Lower joint 1 limit reached"
            self.qdot.command = np.zeros(7)
            self.qdot.command[0] = self.drive_right
        elif self.q[0] >= 1.5:
            print "Upper joint 1 limit reached"
            self.qdot.command = np.zeros(7)
            self.qdot.command[0] = self.drive_left

        if self.q[1] <= -2.0:
            print "Lower joint 2 limit reached"
            self.qdot.command = np.zeros(7)
            self.qdot.command[1] = self.drive_right
        elif self.q[1] >= 1.0:
            print "Upper joint 2 limit reached"
            self.qdot.command = np.zeros(7)
            self.qdot.command[1] = self.drive_left

        if self.q[2] <= -3.0:
            print "Lower joint 3 limit reached"
            self.qdot.command = np.zeros(7)
            self.qdot.command[2] = self.drive_right
        elif self.q[2] >= 3.0:
            print "Upper joint 3 limit reached"
            self.qdot.command = np.zeros(7)
            self.qdot.command[2] = self.drive_left

        if self.q[3] <= -0.05:
            print "Lower joint 4 limit reached"
            self.qdot.command = np.zeros(7)
            self.qdot.command[3] = self.drive_right
        elif self.q[3] >= 2.5:
            print "Upper joint 4 limit reached"
            self.qdot.command = np.zeros(7)
            self.qdot.command[3] = self.drive_left

        if self.q[4] <= -3.0:
            print "Lower joint 5 limit reached"
            self.qdot.command = np.zeros(7)
            self.qdot.command[4] = self.drive_right
        elif self.q[4] >= 3.0:
            print "Upper joint 5 limit reached"
            self.qdot.command = np.zeros(7)
            self.qdot.command[4] = self.drive_left

        if self.q[5] <= -1.5:
            print "Lower joint 6 limit reached"
            self.qdot.command = np.zeros(7)
            self.qdot.command[5] = self.drive_right
        elif self.q[5] >= 2.0:
            print "Upper joint 6 limit reached"
            self.qdot.command = np.zeros(7)
            self.qdot.command[5] = self.drive_left

        if self.q[6] <= -3.0:
            print "Lower joint 7 limit reached"
            self.qdot.command = np.zeros(7)
            self.qdot.command[6] = self.drive_right
        elif self.q[6] >= 3.0:
            print "Upper joint 7 limit reached"
            self.qdot.command = np.zeros(7)
            self.qdot.command[6] = self.drive_left

    def stop_oscillations(self, error):     #Check if arm oscillating
        rospy.logwarn("Calculating position norm of error")
        err_point = error[0:3, -1]
        i = 0
        err_norm = 0
        while i<len(err_point):
            err_norm += pow(err_point[i],2)
            i += 1
        err_norm = sqrt(err_norm)

        print err_norm

        if err_norm < 0.05:
            self.qdot.command = [0, 0, 0, 0, 0, 0, 0]
            self.move_done_pub.publish(True)
        return

    def calc_ang_vel(self):
        if self.running_flag:
            print "========================================="
            rospy.logwarn("Calculating joint velocities.")
            #self.running_flag = 0    # Resetting flag

            # Grab M0 and Slist from baxter_right_description.py
            parts = right_char()
            M0 = parts[0] #Zero config of right_hand
            Slist = parts[1] #6x7 screw axes mx of right arm

            #Convert Slist to Blist
            Tbs = r.TransInv(M0)
            Blist = np.zeros(Slist.shape) #6x7 mx of 0's
            for item, screw in enumerate(Slist.T): #index = 7, screw = 6x1
                Blist[:, item] = np.dot(r.Adjoint(Tbs), screw.T)

            #Current joint angles
            with self.mutex:
                q_now = self.q #1x7 mx
            print "q_now", q_now

            # Desired config: base to desired - Tbd
            with self.mutex:
                T_sd = self.T_goal

            # Find transform from current pose to desired pose, error
            #e = Tbd = TbsTsd
            e = np.dot(r.TransInv(r.FKinBody(M0, Blist, q_now)), T_sd)
            print "error", e

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
                qdot_new = 0.1*(qdot_new/scale)*self.joint_vel_limit

            self.qdot.command = qdot_new #1x7
            print "scaled qdot", self.qdot.command

            #Safety checks - will modify self.qdot.command if needed
            #self.check_joint_limits()
            self.stop_oscillations(e)

            self.joint_cmd_pub.publish(self.qdot) # Publishing velocity
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
