#!/usr/bin/env python
import argparse
import rospy
import pyrealsense2 as rs
import cv2
import numpy as np
from detector import get_ball_centers, get_marker_centers
from ballbot.msg import Centroid

SAVED_MARKERS = [[176, 507], [72, 411]]
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


def main(is_visual):
    pub = rospy.Publisher('ball_position', Centroid, queue_size=10)
    rospy.init_node('ball_publisher', anonymous=True)
    rate = rospy.Rate(30)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    def clean_shutdown():
        pipeline.stop()

    rospy.on_shutdown(clean_shutdown)

    # The first frames tend to be blue
    for _ in range(150):
        pipeline.wait_for_frames()

    rospy.loginfo('Running.')

    check_calibration(pipeline)

    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        image = np.asanyarray(color_frame.get_data())

        ball_centers, conts = get_ball_centers(image)

        if is_visual:
            visualize(image, ball_centers, conts)

        if len(ball_centers) > 0:
            pub.publish(Centroid(ball_centers[0][0], ball_centers[0][1]))
        else:
            pub.publish(Centroid(-1, -1))
        rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Ball Grasping')
    parser.add_argument('-b', '--bkg', action='store_true')
    args = parser.parse_args()

    try:
        main(not args.bkg)
    except rospy.ROSInterruptException:
        pass
