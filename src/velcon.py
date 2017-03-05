#!/usr/bin/env python
#-----------------------------------------------------------------------
# Task:
# 1. Finds Js and Vs
# 2. Uses naive least-squares damping to find qdot
#   (Addresses inf vel at singularities)
# 3. Publishes qdot as JointCommand msg to Baxter joint_command topic
#
# Last Updated: 3/3/17
#-----------------------------------------------------------------------
import rospy
import numpy as np
from std_msgs.msg import Bool
from baxter_right_description import right_char
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import modern_robotics as r
from baxter_core_msgs.msg import JointCommand

#Global variables
global desired_pose, current_pose
desired_pose = Pose()
current_pose = Pose()

def getCurrPose(ref):          #Callback stores reference pose
    global current_pose
    current_pose = ref.pose

def getNewPose(goal):          #Callback stores pose published by commander
    global desired_pose
    desired_pose = goal

def desired_avel():            #Calculates desired ang vel
    #Creating publisher for sending new qdot to Baxter
    qdot_pub = rospy.Publisher('robot/limb/right/joint_command', JointCommand, queue_size = 3)

    # Create KDL model
    robot = URDF.from_xml_file("/home/stephanie/Projects/catkin_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")
    kin = KDLKinematics(robot, "base", "right_hand")

    # Defining damping parameter
    damping = rospy.get_param('lambda', '0.005')

    # Grab M0 and Slist from baxter_right_description.py
    parts = right_char()
    M0 = parts[0] #Zero config of right_hand
    Slist = parts[1] #6x7 screw axes mx of right arm

    # Find joint angles and transform mx of current config
    q_now = kin.inverse(current_pose) #1x7 matrix
    T_now = r.FKinSpace(M0, Slist, q_now)

    # Find transform mx to desired_pose
    q_goal = kin.inverse(desired_pose)
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
    invterm = np.linalg.inv(np.dot(Js.T, Js) + pow(damping,2)*np.eye(n))
    qdot = np.dot(np.dot(invterm,Js.T),Vs)

    #Building JointCommand msg for publishing
    qdot_new = JointCommand()
    qdot_new.mode = 2 #Velocity mode
    qdot_new.command = [qdot[0], qdot[1], qdot[2], qdot[3], qdot[4], qdot[5], qdot[6]]
    qdot_new.names = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]

    qdot_pub.publish(qdot_new) #default is 5Hz

if __name__ == '__main__':
    print "Inside velcon.py"
    rospy.init_node('velcon')

    #Creating subscriber for current position
    rospy.Subscriber('/robot/limb/right/endpoint_state', EndPointState, getCurrPose)

    #Creating subscriber for new target position
    rospy.Subscriber('target_position', Pose, getNewPose)

    #Creating publisher for notifying commander of arm status
    movestat_pub = rospy.Publisher('move_done', Bool, queue_size = 3)

    #Create loop which runs desired_avel
    while not rospy.is_shutdown():
        desired_avel()
        move_again = True #Signal that velcon ready to calc for next move
        movestat_pub.Publish(move_again)

    rospy.spin()
