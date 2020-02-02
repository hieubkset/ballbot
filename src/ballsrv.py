#!/usr/bin/env python
import argparse
import rospy
import pyrealsense2 as rs
import cv2
import numpy as np
from detector import get_ball_centers, get_marker_centers
from ballbot.srv import *
from calibration import get_pixel_markers


SAVED_MARKERS = get_pixel_markers()
MARKER_THRES = 5


def get_markers(centers):
    centers = np.array(centers)

    x_min = centers[:, 0].min()
    x_max = centers[:, 0].max()
    y_min = centers[:, 1].min()
    y_max = centers[:, 1].max()

    return x_min, x_max, y_min, y_max


def is_calibrated(markers):
    x_min, x_max, y_min, y_max = markers
    return np.abs(x_min - SAVED_MARKERS[0][0]) < MARKER_THRES \
           and np.abs(x_max - SAVED_MARKERS[0][1]) < MARKER_THRES \
           and np.abs(y_min - SAVED_MARKERS[1][0]) < MARKER_THRES \
           and np.abs(y_max - SAVED_MARKERS[1][1]) < MARKER_THRES


def check_calibration(pipeline):
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    image = np.asanyarray(color_frame.get_data())

    marker_centers = get_marker_centers(image)
    markers = get_markers(marker_centers)
    if not is_calibrated(markers):
        rospy.logwarn('Need calibration. Current markers: (%d, %d, %d, %d)' % markers)
    else:
        rospy.loginfo('Calibrated.')


def visualize(image, centers, conts):
    cv2.drawContours(image, conts, -1, (255, 255, 255), 1)
    if len(centers): cv2.circle(image, centers[0], 1, [255, 255, 0], 20) # mark for the ball published
    cv2.imshow('Ball Grasping', image)
    cv2.waitKey(1)


class BallService:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        # The first frames tend to be blue
        for _ in range(150):
            self.pipeline.wait_for_frames()

    def start_ball_server(self):
        rospy.init_node('ballsrv', anonymous=True)
        rospy.loginfo('Running.')
        check_calibration(self.pipeline)
        rospy.Service('ball_position', Target, self.hand_get_ball)
        rospy.spin()


    def hand_get_ball(self, req):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        image = np.asanyarray(color_frame.get_data())

        ball_centers, conts = get_ball_centers(image)

        if len(ball_centers) > 0:
            target = TargetResponse(ball_centers[0][0], ball_centers[0][1])
        else:
            target = TargetResponse(-1, -1)

        # rospy.loginfo('Return a ball: (%d, %d)' % (target.x, target.y))

        return target

if __name__ == '__main__':
    try:
        ball_service = BallService()
        ball_service.start_ball_server()
    except rospy.ROSInterruptException:
        pass
