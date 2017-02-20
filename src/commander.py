#!/usr/bin/env python

'''
Tasks:
1. Upon startup, continuously publishes home pose
2. When 'move_again' flag is TRUE, continuously issues new pose messages to topic 'target_position'
'''
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
coord = []

def getFlag(flag): #Store arm status Bool into global var move_again
    #flag = TRUE if Baxter is ready for next move
    global move_again
    move_again = flag

if __name__ == '__main__':
    rospy.init_node('commander')

    #Creating publisher for sending out desired configurations
    pub = rospy.Publisher('target_position', Pose, queue_size = 10)

    #Creating subscriber for listening to arm status
    sub = rospy.Subscriber('move_done', Bool, getFlag)

    #Home pose
    home_config=Pose(
    position=Point(
        x=0.656982770038, y=-0.852598021641, z=0.0388609422173,
    ),
    orientation=Quaternion(
        x=0.367048116303, y=0.885911751787, z=-0.108908281936,w=0.261868353356,
    ))

    #Test pose
    test = [0.871394050868,0.00321779590923,-0.06,-0.115830906904,0.0266418026216,0.0784627927274,-0.0197168440116]

    #In future listen for new poses

    #Go home first
    send_config = home_config
    print "Going home"

    while not rospy.is_shutdown():
        #Switch to publishing new_config if old_config reached
        if move_again == True:
            #Store old pose (just in case, not used yet)
            old_config = send_config

            #Setting up new pose for disassembly
            coord = test

            #Disassemble coord pose into Point and Quaternion vectors
            send_config=Pose(
                position=Point(
                    x=coord[0],
                    y=coord[1],
                    z=coord[2],
                ),
                orientation=Quaternion(
                    x=coord[3],
                    y=coord[4],
                    z=coord[5],
                    w=coord[6],
                )
            )

        #Continually publish a pose
        pub.publish(send_config)

    #Prevent Python from exiting unless node stopped
    rospy.spin()

