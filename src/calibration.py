#!/usr/bin/env python
import pyrealsense2 as rs
import numpy as np
import cv2
import json

import rospy
import intera_interface
import intera_external_devices

import rospkg
import os

rospack = rospkg.RosPack()
PACKAGE_LOCATION = rospack.get_path('ballbot')
CALIB_IMAGE = os.path.join(PACKAGE_LOCATION, 'calib/image.png')
CALIB_INFO = os.path.join(PACKAGE_LOCATION, 'calib/info.json')

lower = np.array([20, 180, 130])
upper = np.array([25, 255, 255])


def get_center(x, y, w, h):
    return int(x + 0.5 * w), int(y + 0.5 * h)


def pixel():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    for _ in range(300):
        pipeline.wait_for_frames()

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        image = np.asanyarray(color_frame.get_data())
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        _, conts, h = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        centers = [get_center(*cv2.boundingRect(c)) for c in conts]
        if len(centers) == 4:
            pipeline.stop()

            cv2.drawContours(image, conts, -1, (255, 255, 0), 1)
            for center in centers:
                cv2.circle(image, center, 3, [255, 0, 0])
            cv2.imwrite(CALIB_IMAGE, image)

            centers = np.array(centers)
            x_min = centers[:, 0].min()
            x_max = centers[:, 0].max()
            y_min = centers[:, 1].min()
            y_max = centers[:, 1].max()
            return [[x_min, x_max], [y_min, y_max]]


def cartesian():
    rospy.init_node("move_to_position")
    limb = intera_interface.Limb('right')
    limb.move_to_neutral()

    cars = []
    for i in range(4):
        print("Enter point %d" % (i + 1))
        done = False
        while not done:
            c = intera_external_devices.getch()
            if c in ['\x1b', '\x03']:
                return None
            elif c == 'p':
                done = True
        current_pose = limb.endpoint_pose()
        cars.append([current_pose['position'].x, current_pose['position'].y])
        print('Point %d: (%.2f, %.2f)' % (i + 1, current_pose['position'].x, current_pose['position'].y))

    cars = np.array(cars)
    X_min = cars[:, 0].min()
    X_max = cars[:, 0].max()
    Y_min = cars[:, 1].min()
    Y_max = cars[:, 1].max()

    return [[X_min, X_max], [Y_max, Y_min]]


def get_pixel_markers():
    with open(CALIB_INFO, 'r') as f:
        data = json.load(f)
    return data['pixel']


def get_cartesian_markers():
    with open(CALIB_INFO, 'r') as f:
        data = json.load(f)
    return data['cartesian']


def main():
    print('Get marker pixel position ...')
    pixel_markers = pixel()
    print('Marker pixel = ', pixel_markers)

    print('Get marker catersian position ...')
    cartesian_markers = cartesian()
    print('Marker cartesian = ', cartesian_markers)

    if cartesian_markers:
        print("Save calib info? [y/n]")
        done = False
        while not done:
            c = intera_external_devices.getch()
            if c == 'n':
                done = True
            elif c == 'y':
                calib_info = {'pixel': pixel_markers, 'cartesian': cartesian_markers}
                with open(CALIB_INFO, 'w') as f:
                    json.dump(calib_info, f)
                done = True
                print('Saved calib info at ' + CALIB_INFO)


if __name__ == '__main__':
    main()
