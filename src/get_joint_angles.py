#!/usr/bin/env python
import argparse

import rospy
import intera_interface

from geometry_msgs.msg import Pose
from sawyer_ik import ik

from intera_interface import CHECK_VERSION


def main():
    rospy.init_node("get_joint_angles")

    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    limb = intera_interface.Limb('right')

    joints = limb.joint_names()
    joint_command = {}
    for joint_name in joints:
        current_position = limb.joint_angle(joint_name)
        joint_command[joint_name] = current_position

    print joint_command


if __name__ == '__main__':
    main()