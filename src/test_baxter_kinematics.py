# Script Contains Screw Axes of Baxter's Right Arm
# Last updated: 2/17/17
#-----------------------------------------------------------------------
import tf.transformations as tr
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import robot_calc_functions as mr
import numpy as np

# create KDL model
robot = URDF.from_xml_file("/home/stephanie/Projects/catkin_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")
kin = KDLKinematics(robot, "base", "right_upper_elbow")

# let's build screw axis for first joint: /right_upper_shoulder
w1 = np.array([0,0,1]) #z-axis of SO(3)
q1 = np.array([0.06402724, -0.25902738, 0]) #frame origin
v1 = -np.cross(w1,q1)
S1 = np.append(w1,v1)

# let's build screw axis for second joint: /right_lower_shoulder
w2 = np.array([np.sqrt(2)/2, np.sqrt(2)/2, 0])
q2 = np.array([0.11281752, -0.30781784, 0.399976])
v2 = -np.cross(w2,q2)
S2 = np.append(w2,v2)

# let's build screw axis for third joint: /right_upper_elbow
w3 = np.array([np.sqrt(2)/2, -np.sqrt(2)/2, 0])
q3 = np.array([0.18494228, -0.37994287, 0.399976])
v3 = -np.cross(w3,q3)
S3 = np.append(w3,v3)

#WIP--------------------------------------------------------------
# let's build screw axis for fourth joint: /right_lower_elbow
w4 = np.array([np.sqrt(2)/2, np.sqrt(2)/2, 0])
q4 = np.array([0.3705, -0.5655, 0.33098])
v4 = -np.cross(w4,q4)
S4 = np.append(w4,v4)

# let's build screw axis for /right_upper_forearm
w5 = np.array([np.sqrt(2)/2, -np.sqrt(2)/2, 0])
q5 = np.array([0.44375, -0.63875, 0.33098])
v5 = -np.cross(w5,q5)
S5 = np.append(w5,v5)

# let's build screw axis for /right_lower_forearm
w6 = np.array([np.sqrt(2)/2, np.sqrt(2)/2, 0])
q6 = np.array([0.63516, -0.83017, 0.32098])
v6 = -np.cross(w6,q6)
S6 = np.append(w6,v6)

# let's build screw axis for /right_hand
w7 = np.array([np.sqrt(2)/2, -np.sqrt(2)/2, 0])
q7 = np.array([0.79746, -0.99246, 0.32098])
v7 = -np.cross(w7,q7)
S7 = np.append(w7,v7)

# transform from /base to /right_upper_elbow
M0 = np.array(kin.forward([0,0,0,0,0,0,0]))

Slist = np.array([S1,S2,S3,S4,S5,S6,S7]).T
print Slist

# test forward kinematics:
q = np.random.uniform(-1,1,3)
g_kin = kin.forward(q)
g_mr = mr.FKinSpace(M0, Slist, q)


