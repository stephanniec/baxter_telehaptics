#!/usr/bin/env python
#-----------------------------------------------------------------------
# Tasks:
# 1. Continuously publishes home pose
# 2. When 'MOVE_AGAIN' is TRUE, issues new pose to topic 'target_position'
# 3. Updates 'MOVE_AGAIN' after new pose sent to false
#
# Last updated: 3/20/17
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
MOVE_AGAIN = False
UI_POSE = Pose()

def move_done_cb(move_done_flag):
    give_move_pub = rospy.Publisher('give_move', Bool, queue_size = 3)
    if move_done_flag:
        give_move_pub.publish(True)

def sent_move_cb(sent_move_flag): #Get notification to move from ui
    #TRUE if new move received from ui
    global MOVE_AGAIN
    MOVE_AGAIN = sent_move_flag

def ui_output_cb(pose_from_ui): #Stores new pose in trajectory
    global Ui_pose
    UI_POSE = pose_from_ui

if __name__ == '__main__':
    rospy.init_node('commander')
    rospy.logwarn("Starting up program...")

    #Creating subscriber for listening to UI status
    rospy.Subscriber('sent_move', Bool, sent_move_cb)

    #Creating subscriber for listening to UI poses
    rospy.Subscriber('ui_output', Pose, ui_output_cb)

    #Creating publisher for sending out desired configurations
    target_pub = rospy.Publisher('target_position', Pose, queue_size = 3)

    #Creating publisher for starting velocity control
    start_move_pub = rospy.Publisher('start_move', Bool, queue_size = 3)

    #Home pose - Neutral
    home_config=Pose(
        position = Point(
          x = 0.0,
          y = -0.55,
          z = -0.0,
        ),
        orientation = Quaternion(
          x = 0.75,
          y = 0.0,
          z = 1.26,
          w = 0.0,
    ))

    #Go home first
    send_config = home_config

    rospy.logwarn("Going home.")

    MOVE_AGAIN = True

    while not rospy.is_shutdown():
        #Switch to publishing new_config if old_config reached
        if MOVE_AGAIN == True:
            print "Getting new pose..."
            #Store old pose (just in case...not used yet)
            old_config = send_config

            #Assign new pose from UI for publishing
            send_coord = UI_POSE
            start_move_pub.publish(True)
            MOVE_AGAIN = False
            rospy.logwarn("End of commander while loop")

        target_pub.publish(send_config)

    rospy.spin()

