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

        self.DISTANCE_THRESHOLD = 300
        self.isDark = True

        self.srv = self.create_service(ARCam, 'get_target_pos', self.get_target_pos)
        
    def get_target_pos(self, request, response):
        questImage = self.bridge.imgmsg_to_cv2(request.img_target, desired_encoding='rgb8')
        robotImage = self.bridge.imgmsg_to_cv2(request.camera_view, desired_encoding='rgb8')

        self.get_logger().info('Starting matching')
        object2D_pos, cv_sift = self.combo_test(questImage, robotImage, request.target)
        self.get_logger().info('Finished matching')

        pos = Vector3()
        pos.x = float(object2D_pos[0])
        pos.y = float(object2D_pos[1])
        pos.z = 0.0
        self.get_logger().info(f'Target position: {pos}')

        sift_match_img = self.bridge.cv2_to_imgmsg(cv_sift)

        response.target_pos = pos
        response.sift_match = sift_match_img

        self.get_logger().info(f'Returning response: {response.target_pos}')
        return response

    def combo_test(self, qImg, rImg, objPt):
        selectedPt = np.array([int(objPt.x), int(objPt.y)])

        # run algorithms
        sift_target_keys, sift_target_desc, sift_cam_keys, sift_cam_desc = self.siftTest(qImg, rImg, selectedPt)
        surf_target_keys, surf_target_desc, surf_cam_keys, surf_cam_desc = self.surfTest(qImg, rImg, selectedPt)

        # copy image
        cam_img = qImg.copy()
        target_img = rImg.copy()
        cv2.circle(qImg, selectedPt, 5, color=(0,0,255), thickness=5)

        cam_keys = sift_cam_keys + surf_cam_keys
        target_keys = sift_target_keys + surf_target_keys

        # match
        cv2.drawKeypoints(rImg, cam_keys, cam_img, (0, 255, 0))
        cv2.drawKeypoints(qImg, target_keys, target_img, (0, 255, 0))

        bf = cv2.BFMatcher(cv2.NORM_L2)
        sift_matches = bf.knnMatch(sift_target_desc, sift_cam_desc, k=2)
        surf_matches = bf.knnMatch(surf_target_desc, surf_cam_desc, k=2)

        # get match points of sift and surf to combine
        filtered_sift_matches = ratio_test(sift_matches)
        filtered_surf_matches = ratio_test(surf_matches)

        # combine sift and surf matches 
        matches = list(filtered_sift_matches)
        for match in filtered_surf_matches:
            new_match = cv2.DMatch(
                _queryIdx=match.queryIdx + len(sift_target_keys),
                _trainIdx=match.trainIdx + len(sift_cam_keys),
                _distance=match.distance
            )
            matches.append(new_match)
        matches = sorted(matches, key=lambda x: x.distance)

        # set up RANSAC homography
        target_pts = np.float32([
            target_keys[m.queryIdx].pt
            for m in matches
        ]).reshape(-1, 1, 2)
        cam_pts = np.float32([
            cam_keys[m.trainIdx].pt
            for m in matches
        ]).reshape(-1, 1, 2)

        H, mask = cv2.findHomography(
            target_pts,
            cam_pts,
            cv2.RANSAC,
            3.0,
            maxIters=5000,
            confidence=0.99
        )
        if H is not None:
            filtered_matches = [
                m for i, m in enumerate(matches)
                if mask[i][0] == 1
            ]
        tfPt = transform_point(H, selectedPt)
        cv2.circle(rImg, (int(tfPt[0]), int(tfPt[1])), 5, color=(0,0,255), thickness=5)

        # Draw matches between both images
        final = cv2.drawMatches(
            qImg,
            target_keys,
            rImg,
            cam_keys,
            filtered_matches,
            None,
            matchColor=(0, 255, 0),
            singlePointColor=(255, 0, 0),
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )

        # cv2.imshow(final)
        # cv2.waitKey(0)

        # cv2.destroyWindow()

        return tfPt, final

    def siftTest(self, qImg, rImg, selectedPt):
        sift = cv2.SIFT_create(contrastThreshold=0.03)

        gray_target = cv2.cvtColor(qImg, cv2.COLOR_BGR2GRAY)
        gray_robot = cv2.cvtColor(rImg, cv2.COLOR_BGR2GRAY)

        # draw keypoints on image to compare later
        cam_keys, cam_desc = sift.detectAndCompute(gray_robot, None)
        target_keys, target_desc = sift.detectAndCompute(gray_target, None)

        target_keys = list(target_keys)

        # filter to a selected point with radius threshold
        filtered_target_keys = [] 
        filtered_target_desc = []
        for kp in range(len(target_keys)):
            pt = np.array((target_keys[kp].pt[0], target_keys[kp].pt[1]))
            if np.linalg.norm(pt - selectedPt) < self.DISTANCE_THRESHOLD:
                filtered_target_keys.append(target_keys[kp])
                filtered_target_desc.append(target_desc[kp])
                # filtered_target_desc = np.append(filtered_target_desc, target_keys[kp])
        filtered_target_desc = np.array(filtered_target_desc)
        
        return filtered_target_keys, filtered_target_desc, cam_keys, cam_desc

    def surfTest(self, qImg, rImg, selectedPt):
        hessian = 400 # default val
        surf = cv2.xfeatures2d.SURF_create(hessian)
        surf.setExtended(True) # change dimensions to make it similar to sift

        # grey scale
        gray_target = cv2.cvtColor(qImg, cv2.COLOR_BGR2GRAY)
        gray_cam = cv2.cvtColor(rImg, cv2.COLOR_BGR2GRAY)

        # draw keypoints on image to compare later
        target_keys, target_desc = surf.detectAndCompute(gray_target, None)
        cam_keys, cam_desc = surf.detectAndCompute(gray_cam, None)
        # print(target_desc)

        # keep track of indexes of keys to filter
        filtered_target_keys = [] 
        filtered_target_desc = []
        for kp in range(len(target_keys)):
            pt = np.array((target_keys[kp].pt[0], target_keys[kp].pt[1]))
            if np.linalg.norm(pt - selectedPt) < self.DISTANCE_THRESHOLD:
                filtered_target_keys.append(target_keys[kp])
                filtered_target_desc.append(target_desc[kp])
                # filtered_target_desc = np.append(filtered_target_desc, target_keys[kp])
        filtered_target_desc = np.array(filtered_target_desc)

        return filtered_target_keys, filtered_target_desc, cam_keys, cam_desc

def ratio_test(matches, threshold=0.85):
    good = []

    for pair in matches:
        if len(pair) == 2:
            m, n = pair
            if m.distance < threshold * n.distance:
                good.append(m)

    return good

def transform_point(H, point):      # transform points given homography
    # Convert point into the required format
    point = np.array(
        [[point]],
        dtype=np.float32
    )

    # Transform point
    transformed = cv2.perspectiveTransform(
        point, H
    )

    return transformed[0][0]


def main(args=None):
    rclpy.init(args=args)

    cam_srv = camera_srv()
    rclpy.spin(cam_srv)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
