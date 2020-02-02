#!/usr/bin/env python2
# Modified intera SDK script for interfacing with IK services
import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


def ik(limb, coordinates, orientation):
    angles = limb.joint_angles()
    ns = "ExternalTools/right/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=coordinates.x,
                    y=coordinates.y,
                    z=coordinates.z,
                ),
                orientation=Quaternion(
                    x=orientation.x,
                    y=orientation.y,
                    z=orientation.z,
                    w=orientation.w, ),
            ), ), }

    ikreq.pose_stamp.append(poses['right'])
    ikreq.tip_names.append('right_hand')
    ikreq.seed_mode = ikreq.SEED_USER

    seed = JointState()
    seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
    seed.position = [angles[a] for a in seed.name]
    ikreq.seed_angles.append(seed)

    # Optimize the null space in terms of joint configuration
    ikreq.use_nullspace_goal.append(True)
    goal = JointState()
    goal.name = ['right_j2']
    goal.position = [0]
    ikreq.nullspace_goal.append(goal)
    ikreq.nullspace_gain.append(0.4)

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))

    limb_joints = angles
    # Check if result valid, and type of seed ultimately used to get solution
    if (resp.result_type[0] > 0):
        seed_str = {ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                    }.get(resp.result_type[0], 'None')
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

    return limb_joints
