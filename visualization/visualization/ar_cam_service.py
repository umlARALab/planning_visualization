from stretch_ar.srv import ARCam

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge

from geometry_msgs.msg import Vector3

import numpy as np

class camera_srv(Node):

    def __init__(self):
        super().__init__('camera_srv')
        self.bridge = CvBridge()

        self.srv = self.create_service(ARCam, 'get_target_pos', self.get_target_pos)
        
        
    def get_target_pos(self, request, response):
        self.get_logger().info('Starting SIFT')
        object2D_pos, cv_sift = self.sift_img(request.camera_view, request.img_target)
        self.get_logger().info('Finished SIFT')

        pos = Vector3()
        pos.x = float(object2D_pos[0])
        pos.y = float(object2D_pos[1])
        pos.z = 0.0
        self.get_logger().info(f'Target position: {pos}')

        sift_match_img = self.bridge.cv2_to_imgmsg(cv_sift)

        response.target_pos = pos
        response.sift_match = sift_match_img

        self.get_logger().info(f'Returning response')
        return response

    def sift_img(self, cam_img, targ_img):
        sift = cv2.SIFT_create(contrastThreshold=0.03)

        cv_image = self.bridge.imgmsg_to_cv2(cam_img, 'bgr8')
        target_image = self.bridge.imgmsg_to_cv2(targ_img)

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

        # cv2.circle(final, (int(avg_pt[0]), int(avg_pt[1])), 5, (255, 0, 0), 5)
        # cv2.imshow('stretch', final)
        # cv2.imshow('target keys', )
        # cv2.waitKey(1)

        return avg_pt, final


def main(args=None):
    rclpy.init(args=args)

    cam_srv = camera_srv()
    rclpy.spin(cam_srv)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
