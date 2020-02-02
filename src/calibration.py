#!/usr/bin/env python
import pyrealsense2 as rs
import numpy as np
import cv2

import rospy
import intera_interface
import intera_external_devices

lower = np.array([20, 180, 130])
upper = np.array([25, 255, 255])


def get_center(x, y, w, h):
    return int(x + 0.5 * w), int(y + 0.5 * h)


def pixel():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    cnt = 0
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if cnt < 300:
            cnt += 1
        cnt = 0
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
            cv2.imwrite('calib/c1.png', image)

            centers = np.array(centers)
            x_min = centers[:, 0].min()
            x_max = centers[:, 0].max()
            y_min = centers[:, 1].min()
            y_max = centers[:, 1].max()
            return x_min, x_max, y_min, y_max


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
            if c == 'q' or c == 'p':
                done = True
        current_pose = limb.endpoint_pose()
        cars.append([current_pose['position'].x, current_pose['position'].y])
        print('Point %d: (%.2f, %.2f)' % (i + 1, current_pose['position'].x, current_pose['position'].y))

    cars = np.array(cars)
    X_min = cars[:, 0].min()
    X_max = cars[:, 0].max()
    Y_min = cars[:, 1].min()
    Y_max = cars[:, 1].max()

    return X_min, X_max, Y_min, Y_max


def main():
    x_min, x_max, y_min, y_max = pixel()
    # X_min, X_max, Y_min, Y_max = cartesian()
    pxy = [[x_min, x_max], [y_min, y_max]]
    # cxy = [[X_min, X_max], [Y_max, Y_min]]
    print('pxy = ', pxy)
    # print('cxy = ', cxy)


if __name__ == '__main__':
    main()
