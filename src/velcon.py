#!/usr/bin/env python
#-----------------------------------------------------------------------
# Task:
# 1. Finds Js and Vs
# 2. Uses naive least-squares damping to find qdot
#   (Addresses inf vel at singularities)
# 3. Publishes qdot as JointCommand msg to Baxter joint_command topic
#
# Last Updated: 3/18/17
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

#Global variables
CURRENT_POSE = Pose()
DESIRED_POSE = Pose()
CURRENT_POSE_FLAG = 0
DESIRED_POSE_FLAG = 0

def getCurrPose(ref):          #Callback stores reference pose
    global CURRENT_POSE_FLAG
    CURRENT_POSE = ref.pose
    CURRENT_POSE_FLAG = 1
    #rospy.logwarn("Current pose")
    print "CURRENT_POSE inside callback", CURRENT_POSE

def getNewPose(goal):          #Callback stores pose published by commander
    global DESIRED_POSE_FLAG
    DESIRED_POSE = goal
    DESIRED_POSE_FLAG = 1
    #rospy.logwarn("New pose")
    # print DESIRED_POSE

# def getCurrPose(ref):          #Callback stores reference pose
#     current_pose = Pose()
#     current_pose = ref.pose
#     return current_pose

# def getNewPose(goal):          #Callback stores pose published by commander
#     desired_pose = Pose()
#     desired_pose = goal
#     return desired_pose

def desired_avel():            #Calculates desired ang vel
    rospy.logwarn("Poses inside avel")
    # print "CURRENT_POSE, inside desired angular vel", CURRENT_POSE
    # print "DESIRED_POSE" ,DESIRED_POSE

    #Creating publisher for sending new qdot to Baxter
    qdot_pub = rospy.Publisher('robot/limb/right/joint_command', JointCommand, queue_size = 10)

    # Create KDL model
    robot = URDF.from_xml_file("/home/stephanie/Projects/catkin_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")
    kin = KDLKinematics(robot, "base", "right_hand")

    # Defining damping parameter
    damping = rospy.get_param('lambda', '0.005')

    # Grab M0 and Slist from baxter_right_description.py
    parts = right_char()
    M0 = parts[0] #Zero config of right_hand
    Slist = parts[1] #6x7 screw axes mx of right arm

    rospy.logwarn("Poses inside avel critical")
    # print CURRENT_POSE
    # print DESIRED_POSE

    # Find joint angles and transform mx of current config
    q_now = kin.inverse(CURRENT_POSE) #1x7 matrix
    rospy.logwarn("q_now")
    print q_now
    T_now = r.FKinSpace(M0, Slist, q_now)

    # Find transform mx to DESIRED_POSE
    q_goal = kin.inverse(DESIRED_POSE)
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
    invterm = np.linalg.inv(np.dot(Js.T, Js) + pow(damping, 2)*np.eye(n))
    qdot = np.dot(np.dot(invterm,Js.T),Vs)

    #Preparing to send new angular velocities
    limb = baxter_interface.Limb('right')   # Instance of limb class
    qdot_names = kin.get_joint_names()

    rospy.logwarn("qdot_names")
    print qdot_names

    qdot_new = dict(zip(qdot_names,qdot))
    limb.set_joint_velocities(qdot_new)

    # #Building JointCommand msg for publishing
    # qdot_new = JointCommand()
    # qdot_new.mode = 2 #Velocity mode
    # qdot_new.command = [qdot[0], qdot[1], qdot[2], qdot[3], qdot[4], qdot[5], qdot[6]]
    # qdot_new.names = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]

    qdot_pub.publish(qdot_new) #default is 5Hz
    rospy.logwarn("Sending new pose")

if __name__ == '__main__':
    rospy.init_node('velcon')
    rospy.logwarn("Inside velcon")

    #Creating subscriber for current position
    rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, getCurrPose)

    #Creating subscriber for new target position
    rospy.Subscriber('target_position', Pose, getNewPose)

    #Creating publisher for notifying commander of arm status
    movestat_pub = rospy.Publisher('move_done', Bool, queue_size = 10)

    #Create loop which runs desired_avel
    while not rospy.is_shutdown():
        if DESIRED_POSE_FLAG == 1:
            rospy.logwarn("After main condition")
            # print CURRENT_POSE
            # print DESIRED_POSE
            print "DESIRED_POSE_FLAG", DESIRED_POSE_FLAG
            print "CURRENT_POSE_FLAG", CURRENT_POSE_FLAG
            desired_avel()
            move_again = True        #velcon ready to calc for next move
            movestat_pub.publish(move_again)

    rospy.spin()
