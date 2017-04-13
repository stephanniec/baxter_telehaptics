#!/usr/bin/env python
#----------------------------------------------------------------------
# Tasks:
# 1. Publishes the workspace limits of Baxter's right arm as a matrix
#    to 'visualization_marker' for viewing in rviz
#
# Last updated 4/11/17
#----------------------------------------------------------------------
import rospy
from visualization_msgs.msg import Marker

####################
# GLOBAL VARIABLES #
####################
XYZ_MIN = np.array([0.6, -0.75, -0.25])
XYZ_MAX = np.array([0.8, -0.25, 0.25])

class CreateBoundary():
    def __init__(self):
        #Publisher
        self.ws_pub = rospy.Publisher('visualization_marker', Marker, queue_size=3)

        self.rate = rospy.Rate(20)
        self.marker = Marker()

        while not rospy.is_shutdown():
            self.ws_pub.publish()
            self.rate.sleep()

    def getCubePoints(self):



def main():
    rospy.init_node('draw_workspace')

    try:
        cube_ws = CreateBoundary()
    except: rospy.ROSInterruptException: pass

if __name__ == '__main__':
    main()
