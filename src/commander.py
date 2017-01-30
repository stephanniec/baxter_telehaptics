#!/usr/bin/env python

'''
Tasks:
1. Publishes home pose
2. Issues subsequent pose messages to 'target_position' whenever 'move_again' is TRUE
'''

import rospy
from std_msgs.msg import Header, String, Bool
from geometry_msgs.msg import(
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

#Global variable
move_again = None
coord = []

def getFlag(flag):
    #Callback fxn stores arm status Bool into global var move_again
    #flag = TRUE if Baxter is waiting for next move
    global move_again
    move_again = flag

if __name__ == '__main__':
    rospy.init_node('commander')

    #Creating publisher for sending out desired configurations
    pub = rospy.Publisher('target_position', Pose, queue_size = 10)

    #Creating subscriber for listening to arm status
    sub = rospy.Subscriber('move_done', Bool, getFlag)

    #Defining home position
    home_config=Pose(
        position=Point(
            x= 0.464103257539,
            y= 0.12055341652,
            z= -0.0435690126601,
        ),
        orientation=Quaternion(
            x= 1.0,
            y= 0.0,
            z= 0.0,
            w= 0.0,
        ))
    #Publishing home_config
    pub.publish(home_config)
    print "Going home:"
    print home_config

    #Test positions
    test1 = [0.72099597135,0.00972967930975,-0.07, 0.997963425222, 0.0110519031264,0.0102812076764,-0.0619770451548]
    test2 = [0.711382933155,0.169900893084,-0.06,0.996752095131,0.0399645409214,-0.0107386476991,-0.0690852934121]

    #Only publish new config if arm done moving to target position
    if move_again == 1:
        print "Ready for new target position..."
        #New config position
        for i in range(0,2):
            if i == 0:
                coord = test1
                print "Going to test 1:"
                print coord
            else:
                coord = test2
                print "Going to test 2:"
                print coord

        new_config=Pose(
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
        ))
        pub.publish(new_config)
        print "Confirming new_config coordinates:"
        print new_config
    else:
        print "System is busy..."

    rospy.spin()

