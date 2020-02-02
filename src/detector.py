import cv2
import numpy as np

RED_LOWER = np.array([1, 135, 110])
RED_UPPER = np.array([10, 255, 255])

GREEN_LOWER = np.array([35, 80, 50])
GREEN_UPPER = np.array([50, 255, 255])

BLUE_LOWER = np.array([95, 110, 50])
BLUE_UPPER = np.array([115, 255, 255])

YELLOW_LOWER = np.array([20, 180, 130])
YELLOW_UPPER = np.array([30, 255, 255])

KERNELOPEN = np.ones((10, 10))
KERNELCLOSE = np.ones((10, 10))


def get_center(x, y, w, h):
    return int(x + 0.5 * w), int(y + 0.5 * h)


def get_mask(hsv, lower, upper):
    mask = cv2.inRange(hsv, lower, upper)
    mask_open = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNELOPEN)
    mask_close = cv2.morphologyEx(mask_open, cv2.MORPH_CLOSE, KERNELCLOSE)

    return mask_close


def get_ball_centers(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # print(hsv[237, 413])

    red_mask = get_mask(hsv, RED_LOWER, RED_UPPER)
    green_mask = get_mask(hsv, GREEN_LOWER, GREEN_UPPER)
    blue_mask = get_mask(hsv, BLUE_LOWER, BLUE_UPPER)
    mask = red_mask + green_mask + blue_mask

    # countour
    _, conts, h = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # center
    centers = [get_center(*cv2.boundingRect(c)) for c in conts]

    return centers, conts


def get_marker_centers(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = get_mask(hsv, YELLOW_LOWER, YELLOW_UPPER)
    _, conts, h = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    markers = [get_center(*cv2.boundingRect(c)) for c in conts]
    return markers


if __name__ == '__main__':
    image = cv2.imread('image/ex2.png')
    centers, conts = get_ball_centers(image)

    cv2.drawContours(image, conts, -1, (255, 255, 0), 1)
    for center in centers:
        cv2.circle(image, center, 3, [255, 0, 0])
    cv2.imshow('mage', image)
    cv2.waitKey(0)
