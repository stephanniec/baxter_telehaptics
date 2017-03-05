#!/usr/bin/env python
#-----------------------------------------------------------------------
#Tasks:
#1. Continuously publishes home pose
#2. When 'move_again' flag is TRUE, continuously issues new pose msgs to topic 'target_position'
#-----------------------------------------------------------------------

import rospy
import baxter_interface
from std_msgs.msg import Header, String, Bool
from geometry_msgs.msg import(
    Pose,
    Point,
    Quaternion,
)

#Global variable
move_again = False
ui_pose

def getFlag(flag): #Store arm status Bool into global var move_again
    #flag = TRUE if Baxter is ready for next move
    global move_again
    move_again = flag

def getUIPose(from_ui):
    global ui_pose
    ui_pose = from_ui

if __name__ == '__main__':
    rospy.init_node('commander')

    #Creating subscriber for listening to UI poses
    rospy.Subscriber('ui_output', Pose, getUIPose)

    #Creating subscriber for listening to arm status
    rospy.Subscriber('move_done', Bool, getFlag)

    #Creating publisher for sending out desired configurations
    target_pub = rospy.Publisher('target_position', Pose, queue_size = 10)

    #Creating publisher for updating arm status
    update_pub = rospy.Publisher('move_done', Bool, queue_size = 3)

    #Home pose
    home_config=Pose(
    position=Point(
        x=0.656982770038, y=-0.852598021641, z=0.0388609422173,
    ),
    orientation=Quaternion(
        x=0.367048116303, y=0.885911751787, z=-0.108908281936,w=0.261868353356,
    ))

    #Go home first
    send_config = home_config
    print "Going home"

    while not rospy.is_shutdown():
        #Switch to publishing new_config if old_config reached
        if move_again == True:
            #Store old pose (just in case...not used yet)
            old_config = send_config

            #Assign new pose from UI for publishing
            send_coord = ui_pose

        #Continually publish a pose
        target_pub.publish(send_config)

        #Resetting flag
        move_again = False
        update_pub.publish(move_again)

    #Prevent Python from exiting unless node stopped
    rospy.spin()

