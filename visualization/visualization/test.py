import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
import numpy as np

from stretch_ar.msg import ImageTarget
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image


def siftTest():
    sift = cv2.SIFT_create(contrastThreshold=0.03)

    cv_image = cv2.imread('questSampleImg.jpg')
    target_image = cv2.imread('targetCrop.png')

    width, height = target_image.shape[:2]
    image_scale = 1
    if height > width:
        image_scale = int(240 / (height))
    else:
        image_scale = int(240 / (width))

    # target_image = cv2.resize(target_image, None, fx=image_scale, fy=image_scale, interpolation=cv2.INTER_LINEAR)
    cv_image = cv2.resize(cv_image, (1280, 960))

    # set up images
    gray_cam = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    gray_target = cv2.cvtColor(target_image, cv2.COLOR_BGR2GRAY)

    # draw keypoints on image to compare later
    cam_keys, cam_desc = sift.detectAndCompute(gray_cam, None)
    target_keys, target_desc = sift.detectAndCompute(gray_target, None)

    cam_key_img = cv_image.copy()
    target_key_img = target_image.copy()

    cv2.drawKeypoints(cv_image, cam_keys, cam_key_img, (0, 255, 0))
    cv2.drawKeypoints(target_image, target_keys, target_key_img, (0, 255, 0))

    # match keypoints
    if len(cam_desc) != 0 and len(target_desc) != 0:
        brute_matcher = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
        matches = brute_matcher.match(cam_desc, target_desc)
        matches = sorted(matches, key=lambda x: x.distance)

    # filter outliers
    match_pts = []
    for match in matches:
        x, y = cam_keys[match.queryIdx].pt

        match_pts.append([x,y])
    avg_pt = np.mean(match_pts, axis=0)
    match_threshold = 100

    i = 0
    while i < len(matches):
        if np.linalg.norm(match_pts[i] - avg_pt) > match_threshold:
            matches.pop(i)
            match_pts.pop(i)
        else:
            i +=1  
    avg_pt = np.mean(match_pts, axis=0)
    final = cv2.drawMatches(cv_image, cam_keys, gray_target, target_keys, matches, gray_target, flags=2)

    cv2.circle(final, (int(avg_pt[0]), int(avg_pt[1])), 1, (255, 0, 0), 1)

    cv2.imshow('stretch', final)
    # cv2.imshow('here', cam_key_img)
    cv2.waitKey(1)

def main():
    while True:
        siftTest()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()