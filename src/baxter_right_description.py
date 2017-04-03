#!/usr/bin/env python
#----------------------------------------------------------------------
# Tasks:
# Script constructs M0 and Slist for baxter's right arm
# Last updated: 4/2/17
#-----------------------------------------------------------------------
# Python Imports
import numpy as np
import tf.transformations as tr

# ROS Imports
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics

# Local Imports
import modern_robotics as r
import custom_logging as cl

def right_char():

    # Create KDL model
    with cl.suppress_stdout_stderr():
        robot = URDF.from_parameter_server()
    kin = KDLKinematics(robot, "base", "right_hand")

    # Screw axis for joint 1: /right_upper_shoulder
    w1 = np.array([0,0,1]) #z-axis of SO(3)
    q1 = np.array([0.06402724, -0.25902738, 0]) #frame origin
    v1 = -np.cross(w1,q1)
    S1 = np.append(w1,v1)

    # Screw axis for joint 2: /right_lower_shoulder
    w2 = np.array([np.sqrt(2)/2, np.sqrt(2)/2, 0])
    q2 = np.array([0.11281752, -0.30781784, 0.399976])
    v2 = -np.cross(w2,q2)
    S2 = np.append(w2,v2)

    # Screw axis for joint 3: /right_upper_elbow
    w3 = np.array([np.sqrt(2)/2, -np.sqrt(2)/2, 0])
    q3 = np.array([0.18494228, -0.37994287, 0.399976])
    v3 = -np.cross(w3,q3)
    S3 = np.append(w3,v3)

    # Screw axis for joint 4: /right_lower_elbow
    w4 = np.array([np.sqrt(2)/2, np.sqrt(2)/2, 0])
    q4 = np.array([0.3705009, -0.56550217, 0.330976])
    v4 = -np.cross(w4,q4)
    S4 = np.append(w4,v4)

    # Screw axis for joint 5: /right_upper_forearm
    w5 = np.array([np.sqrt(2)/2, -np.sqrt(2)/2, 0])
    q5 = np.array([0.44374996, -0.63875149, 0.330976])
    v5 = -np.cross(w5,q5)
    S5 = np.append(w5,v5)

    # Screw axis for joint 6: /right_lower_forearm
    w6 = np.array([np.sqrt(2)/2, np.sqrt(2)/2, 0])
    q6 = np.array([0.63516341, -0.83016565, 0.320976])
    v6 = -np.cross(w6,q6)
    S6 = np.append(w6,v6)

    # Screw axis for joint 7: /right_wrist
    w7 = np.array([np.sqrt(2)/2, -np.sqrt(2)/2, 0])
    q7 = np.array([0.71716997, -0.91217251, 0.320976])
    v7 = -np.cross(w7,q7)
    S7 = np.append(w7,v7)

    # Transform from /base to /right_hand
    M0 = np.zeros((4,4))
    M0[0:3,0:3] = np.array([[ 0, np.sqrt(2)/2, np.sqrt(2)/2],
                            [ 0, np.sqrt(2)/2, -np.sqrt(2)/2],
                            [-1, 0, 0]])
    M0[0:3,3] = np.array([0.7974618, -0.99246463, 0.320976]) #ee pos @ zero
    M0[3,3] = 1
    Slist = np.array([S1,S2,S3,S4,S5,S6,S7]).T #6x7 matrix

    parts = [M0, Slist]

    return parts

