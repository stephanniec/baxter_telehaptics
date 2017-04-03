#!/usr/bin/env python
#-----------------------------------------------------------------------
# Tasks:
# 1. Continuously publishes home pose
# 2. When 'MOVE_AGAIN' is TRUE, issues new pose to topic 'target_position'
# 3. Updates 'MOVE_AGAIN' after new pose sent to false
#
# Last updated: 3/30/17
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

def startup():
    print "\n*******************************************"
    print "BAXTER-OMNI TELEHAPTICS FEEDBACK CONTROLLER"
    print "*******************************************\n"

    input = raw_input("Press 's' to start program.\n")
    while input != 's':
        print "Invalid input. Please try again.\n"
        input = raw_input("Press 's' to start program.\n")

    print "Starting program."

    # Add user instructions here

if __name__ == '__main__':
    rospy.init_node('commander')

    #Creating subscriber for listening to UI status
    rospy.Subscriber('sent_move', Bool, sent_move_cb)

    #Creating subscriber for listening to UI poses
    rospy.Subscriber('ui_output', Pose, ui_output_cb)

    #Creating publisher for sending out desired configurations
    target_pub = rospy.Publisher('target_position', Pose, queue_size = 3)

    #Creating publisher for starting velocity control
    start_move_pub = rospy.Publisher('start_move', Bool, queue_size = 3)

    startup()

    rospy.logwarn("Storing home pose.")
    #Home pose - Neutral
    home_config=Pose(
        position = Point(
          x = 0.562788881173,
          y = -0.552811513529,
          z = 0.135345612716,
        ),
        orientation = Quaternion(
          x = 0.972272957345,
          y = 0.218776916338,
          z = 0.0824998598396,
          w = 0.00396615922789,
    ))

    #Go home first
    send_config = home_config

    while not rospy.is_shutdown():
        if MOVE_AGAIN == True:
            print "Getting new pose..."

            #Assign new pose from UI for publishing
            send_coord = UI_POSE
            start_move_pub.publish(MOVE_AGAIN)  #Publishes True
            MOVE_AGAIN = False    #Resetting flag to velocity controller
            rospy.logwarn("End of commander while loop")

        target_pub.publish(send_config)    #Publishes False
        start_move_pub.publish(MOVE_AGAIN)

    rospy.spin()

