#!/usr/bin/env python
#----------------------------------------------------------------------
# Framework derived from RSDK Inverse Kinematics Example

# Tasks:
#  1. Listens to topic 'target_position' for a Pose msg
#  2. Calls Baxter's ikservice to find joint angles for new Pose
#  3. Publishes arm status using arm_flag

# Last updated: 2/8/17
#----------------------------------------------------------------------
import rospy
import struct
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import(
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from baxter_core_msgs.srv import(
    SolvePositionIK,
    SolvePositionIKRequest,
)

#Global variables
global desired_pose
desired_pose = Pose()

def getNewPose(msg): #Callback fxn stores pose published by commander.py
    global desired_pose
    desired_pose = msg
    #rospy.loginfo("Received new pose")

def ikmove(limb, pose):
    rospy.loginfo("Inside ikmove")
    print desired_pose

    move_done = False

    rospy.init_node("iksol")

    #Creating publisher for sending out arm status
    pub_arm = rospy.Publisher('move_done', Bool, queue_size = 10)

    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    arm_poses = {
        #Only using right arm
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.657579481614,
                    y=0.851981417433,
                    z=0.0388352386502,
                ),
                orientation=Quaternion(
                    x=-0.366894936773,
                    y=0.885980397775,
                    z=0.108155782462,
                    w=0.262162481772,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=pose.position.x, y=pose.position.y, z=pose.position.z),
                orientation=Quaternion(
                    x=pose.orientation.x, y=pose.orientation.y, z=pose.orientation.z, w=pose.orientation.w),
            ),
        ),
    }

    ikreq.pose_stamp.append(arm_poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.loginfo("Got to here")
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))

        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

        limb = baxter_interface.Limb('right')
        limb.move_to_joint_positions(limb_joints)
        #Waits until reported joint state matches that specified

        move_done = True

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        move_done = False

    pub_arm.publish(move_done)
    return 0

if __name__ == '__main__':
    print "Inside iksol.py"

    #Only using right arm
    rt_limb = 'right'

    #Creating subscriber for listening to new target position
    rospy.Subscriber('target_position', Pose, getNewPose)

    while not rospy.is_shutdown():
        ikmove(rt_limb, desired_pose)


