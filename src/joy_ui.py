#!/usr/bin/env python
#----------------------------------------------------------------------
# Tasks:
# 1. Takes PS3 joy node output and
# 2. Maps controller commands to a pose
# 3. Defines EE pose work space
#
# Last updated 4/11/17
#----------------------------------------------------------------------
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Joy

class SimpleTargets(object):
    def __init__(self):
        rospy.logwarn("Creating SimpleTargets class")

        #Subscribers and Publishers
        self.joy_sub = rospy.Subscriber('joy', Joy, joy_cb)
        self.ref_pose_pub = rospy.Publisher('ref_pose', Pose, queue_size = 3)

        self.r = rospy.Rate(30)    #30Hz
        while not rospy.is_shutdown():
            self.target_pose()
            self.r.sleep()

    def target_pose(self):
        coord = Pose()

        coord = Pose(
          position = Point(
            x = 0.676187476661,
            y = -0.365000752298,
            z = -0.0761134297063,
          ),
          orientation = Quaternion(
            x = -0.185702483327,
            y = 0.959179046594,
            z = 0.201059377568,
            w = 0.0711707169568,
        ))

        self.ref_pose_pub.publish(coord)    # Send move

def main():
    rospy.init_node('joy_ui')

    try:
        new_pose = SimpleTargets()
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__ == '__main__':
    main()
