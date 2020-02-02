#!/usr/bin/env python

import rospy
import intera_interface

from geometry_msgs.msg import Pose
from ballbot.srv import Target, TargetResponse, TargetRequest
from sawyer_ik import ik

from intera_interface import CHECK_VERSION

PIXEL_MARKERS = [[176, 507], [72, 411]]  ## [[x_min, x_max], [y_min, y_max]]
CARTESIAN_MARKERS = [[0.4004000599051002, 0.7964501196045618],
                     [0.22256176082579648, -0.1829347509717589]]  ## [[X_min, X_max], [Y_min, Y_max]]

UP_DROP_POS = {'right_j6': 2.216546875, 'right_j5': 1.032203125, 'right_j4': 0.4072666015625,
               'right_j3': 1.030751953125, 'right_j2': -0.3963544921875, 'right_j1': -0.4486357421875,
               'right_j0': -0.64492578125}

DROP_POS = {'right_j6': 2.215087890625, 'right_j5': 0.8882197265625, 'right_j4': 0.394923828125,
            'right_j3': 1.0808046875,
            'right_j2': -0.3414560546875, 'right_j1': -0.3115224609375, 'right_j0': -0.644205078125}

WAITTING_POS = {'right_j6': 2.376990234375, 'right_j5': 1.3561513671875, 'right_j4': 0.2146865234375,
                'right_j3': 0.5053369140625, 'right_j2': -0.277380859375, 'right_j1': -0.250935546875,
                'right_j0': -0.7421611328125}

# without sleeping, robot moves too fast and fail to grasp balls
# need to sleep a while to be able grasp balls
# Nho cung ko dc, ma lon cung ko dc ? can tim hieu nguyen nhan
SLEEP_TIME = 2


class BallBot:
    def __init__(self):
        self.limb = intera_interface.Limb('right')
        self.limb.move_to_neutral()
        self.limb.set_joint_position_speed(0.15)
        self.gripper = intera_interface.Gripper("right_gripper")
        self.gripper.open()
        self.desired_pose = Pose()
        current_pose = self.limb.endpoint_pose()
        self.desired_pose.orientation = current_pose['orientation']
        self.target_srv = rospy.ServiceProxy("/ball_position", Target)
        self.target_srv.wait_for_service()

    def act(self):
        self.no_ball_cnt = 0
        while not rospy.is_shutdown():
            target = self.target_srv.call(TargetRequest())
            x, y = target.x, target.y
            if x != -1 and y != -1:
                if self.no_ball_cnt > 0:
                    # wait for stable
                    self.no_ball_cnt = 0
                    rospy.sleep(10)
                    self.limb.move_to_neutral()
                else:
                    rospy.loginfo('Picking up a ball at the position: (%d, %d)' % (x, y))
                    cx, cy = self.pixels_to_cartesian(x, y)
                    self.move(cx, cy)
                    self.grasp()
                    self.assemble()
            else:
                if self.no_ball_cnt < 100000: self.no_ball_cnt += 1

    def move(self, x, y):
        self.desired_pose.position.x = x
        self.desired_pose.position.y = y
        self.desired_pose.position.z = -0.08 + 0.40
        joint_angles = ik(self.limb, self.desired_pose.position, self.desired_pose.orientation)
        self.limb.move_to_joint_positions(joint_angles)
        rospy.sleep(SLEEP_TIME)

        self.desired_pose.position.z = -0.08 + 0.11
        joint_angles = ik(self.limb, self.desired_pose.position, self.desired_pose.orientation)
        self.limb.move_to_joint_positions(joint_angles)
        rospy.sleep(SLEEP_TIME)

    def grasp(self):
        self.gripper.set_position(0.0)

    def assemble(self):
        self.limb.move_to_joint_positions(UP_DROP_POS)
        rospy.sleep(SLEEP_TIME)
        self.limb.move_to_joint_positions(DROP_POS)
        rospy.sleep(SLEEP_TIME)
        self.gripper.open()
        self.limb.move_to_joint_positions(WAITTING_POS)

    @staticmethod
    def pixels_to_cartesian(x, y):
        cx = CARTESIAN_MARKERS[0][0] + (x - PIXEL_MARKERS[0][0]) * (
                CARTESIAN_MARKERS[0][1] - CARTESIAN_MARKERS[0][0]) / (PIXEL_MARKERS[0][1] - PIXEL_MARKERS[0][0])
        cy = CARTESIAN_MARKERS[1][0] + (y - PIXEL_MARKERS[1][0]) * (
                CARTESIAN_MARKERS[1][1] - CARTESIAN_MARKERS[1][0]) / (PIXEL_MARKERS[1][1] - PIXEL_MARKERS[1][0])
        return cx, cy


def ball_grasping():
    current_pose = limb.endpoint_pose()
    desired_pose = Pose()
    desired_pose.orientation = current_pose['orientation']

    target_srv = rospy.ServiceProxy("/get_target", target)
    target_srv.wait_for_service()

    while True:
        req = targetRequest()
        req.data = 0
        resp = target_srv.call(req)


def main():
    rospy.init_node("ballgrasp")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    rs.enable()
    ballbot = BallBot()
    ballbot.act()


if __name__ == '__main__':
    main()
