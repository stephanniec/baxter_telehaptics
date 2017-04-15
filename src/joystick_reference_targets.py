#!/usr/bin/env python
#----------------------------------------------------------------------
# Tasks:
# 1. Reads PS3 joy node output
# 2. Maps controller positions to a pose
# 3. Defines EE pose work space
#
# Last updated 4/11/17
#----------------------------------------------------------------------
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Joy
import tf
import tf.transformations as tr

import numpy as np

def calc_quat(th):
    return tr.quaternion_from_euler(3.115649698300095, -0.02399721058403635, th, 'rxyz')

####################
# GLOBAL VARIABLES #
####################
ZERO_POS = np.array([0.902160877388, -0.0715308389238, 0.284243124085])
# ZERO_ORI = np.array([0.997, -0.076, -0.005, -0.002])
ZERO_ORI = calc_quat(0.0)
XYZ_MIN = np.array([0.4, -0.4, 0.01])
XYZ_MAX = np.array([0.82, 0.4, 0.25])
XYZ_SCALE = np.array([0.1, 0.25, 0.25])
XYZ_INVERT_MAP = np.array([-1, -1, 1])
XYZ_INDEX_ARR = np.array([3,0,1])
TH_SCALE = 0.75
TH_INDEX = 2
TH_MIN = -np.pi/2
TH_MAX = np.pi/2
FREQUENCY = 20
DEADMAN_INDEX = 10
REF_FRAME = "base"
TARGET_FRAME = "target"

class SimpleTargets(object):
    def __init__(self):
        rospy.loginfo("Creating SimpleTargets class")

        # setup local variables:
        self.position = ZERO_POS
        self.orientation = ZERO_ORI
        self.ee_dot = np.array([0,0,0])
        self.th = 0.0
        self.th_dot = 0.0

        #Subscribers and Publishers
        self.br = tf.TransformBroadcaster()
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_cb)
        self.ref_pose_pub = rospy.Publisher('ref_pose', Pose, queue_size = 3)
        self.integrate_and_pub_timer = rospy.Timer(rospy.Duration(1/float(FREQUENCY)), self.timer_cb)

        return

    def joy_cb(self, joy_message):
        if joy_message.buttons[DEADMAN_INDEX]:
            # set velocity:
            jvels = XYZ_INVERT_MAP*XYZ_SCALE*np.choose(XYZ_INDEX_ARR, joy_message.axes)
            self.ee_dot = np.array(jvels)
            self.th_dot = TH_SCALE*joy_message.axes[TH_INDEX]
        else:
            self.ee_dot = np.array([0,0,0])
            self.th_dot = 0.0
        return

    def timer_cb(self, event):
        # integrate our goal pose:
        new_pos = self.position + self.ee_dot*(1/float(FREQUENCY))
        # make sure nominal pose is within limits:
        new_pos = np.clip(new_pos, XYZ_MIN, XYZ_MAX)
        # update position:
        self.position = new_pos

        # update orientation
        self.th = np.clip(self.th + self.th_dot*(1/float(FREQUENCY)), TH_MIN, TH_MAX)
        self.orientation = calc_quat(self.th)

        # publish Pose and send tf
        self.ref_pose_pub.publish(Pose(Point(*self.position), Quaternion(*self.orientation)))
        self.br.sendTransform(self.position, self.orientation, rospy.Time.now(), TARGET_FRAME, REF_FRAME)
        return


def main():
    rospy.init_node('joystick_reference_pose_generator')

    try:
        new_pose = SimpleTargets()
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__ == '__main__':
    main()
