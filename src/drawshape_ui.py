#!/usr/bin/env python
#----------------------------------------------------------------------
# Tasks:
# 1. Listens to topic 'move_done'
# 2. Continually publishes poses to make Baxter's arm draw a "square"
#
# Last updated 3/30/17
#----------------------------------------------------------------------

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion

#Global
NEW_MOVE = False

def give_move_cb(give_move_flag): #Get status of commander
    #True if commander ready for next move
    global NEW_MOVE
    NEW_MOVE = give_move_flag

def draw_circle(index):
    #Creating publisher for trajectory
    ui_pub = rospy.Publisher('ui_output', Pose, queue_size = 3)
    #Creating publisher for telling command to start velocity control
    sent_move_pub = rospy.Publisher('sent_move', Bool, queue_size = 3)

    coord = Pose()

    if NEW_MOVE == True:
        if index == 0:
            rospy.logwarn("Point 1")
            coord = Pose(
              position = Point(
                x = 0.628641325891,
                y = -0.509010703442,
                z = 0.144149113727,
              ),
              orientation = Quaternion(
                x = 0.728588116532,
                y = -0.683643597022,
                z =-0.0421406782689,
                w = -0.00386677315367,
            ))

        elif index == 1:
            rospy.logwarn("Point 2")
            coord = Pose(
              position = Point(
                x = 0.734199432303,
                y = -0.46897779467,
                z = 0.121092198119,
              ),
              orientation = Quaternion(
                x = -0.702558430471,
                y = 0.71109071589,
                z = -0.0273469079932,
                w = 0.00371378129335,
            ))

        elif index == 2:
            rospy.logwarn("Point 3")
            coord = Pose(
              position = Point(
                x = 0.628641325891,
                y = -0.509010703442,
                z = 0.144149113727,
              ),
              orientation = Quaternion(
                x = 0.728588116532,
                y = -0.683643597022,
                z =-0.0421406782689,
                w = -0.00386677315367,
            ))

        ui_pub.publish(coord)            # Sending move
        sent_move_flag.publish(True)     # Tell cmd move was sent
        index = index + 1

if __name__ == '__main__':
    rospy.init_node('drawshape_ui')

    #Creating subscriber for listening to arm status
    rospy.Subscriber('give_move', Bool, give_move_cb)

    point_index = 0
    while not rospy.is_shutdown():
        draw_circle(point_index)

    rospy.spin()
