import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
import numpy as np

from stretch_ar.msg import ImageTarget, RobotStatus
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from stretch_ar.srv import ARCam

class ARCamera(Node):
    def __init__(self):
        super().__init__('ar_camera')

        self.bridge = CvBridge()

        self.aruco_marker_sub = self.create_subscription(
            ImageTarget,
            '/quest_camera',
            self.quest_cam_callback,
            10
        )

        self.stretch_camera_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            10
        )

        self.robot_status = self.create_subscription(
            RobotStatus,
            'robot_feedback',
            self.status_callback,
            10
        )

        self.state = 0
        self.target_image = None
        self.camera_image = None

    def quest_cam_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg.image, 'bgr8')
        # cv2.circle(cv_image, (int(msg.position.x), int(msg.position.y)), 5, color=(0, 255, 0), thickness=-1)
        screen_target = np.array((msg.position.x, msg.position.y, msg.position.z))

        contours = self.get_contours(cv_image)

        # go through edges and find target
        closest_contour, centroid = self.find_blob(screen_target, contours)

        # copy image shows contours for debugging and testing purposes
        copy_image = cv_image.copy()
        cv2.drawContours(copy_image, contours, -1, (0, 255, 0), 1)
        cv2.drawContours(copy_image, closest_contour, -1, (0, 0, 255), 3)

        # crop the potential target object 
        box = cv2.boundingRect(closest_contour)
        x, y, w, h = box
        w_padding = int(w *0.25)
        h_padding = int(h * 0.25)
        x -= w_padding
        y -= h_padding
        image_scale = 1

        # scale cropped image to make it more viewable
        if h > w:
            image_scale = int(240 / (h + h_padding))
        else:
            image_scale = int(240 / (w + w_padding))

        self.target_image = cv_image[y:(y+h+(h_padding*2)), x:(x+w+(w_padding*2))]
        big_target_image = cv2.resize(self.target_image, None, fx=image_scale, fy=image_scale, interpolation=cv2.INTER_LINEAR)
                
        # cv2.imshow('Contours', copy_image)
        cv2.imshow('Target', big_target_image)

        cv2.waitKey(1)

    def status_callback(self, msg):
        self.state = msg.state

    # this is where we will be performing SIFT
    def camera_callback(self, msg):
        # ensure image exists
        if self.state == 2:
            self.camera_image = msg

    # apply edge detection and get outlines of image
    def get_contours(self, img):
        upper = 120
        lower = 40

        # get canny image to get outlines
        cv_canny = cv2.addWeighted(img, 1.5, np.zeros(img.shape, img.dtype), 0, 0)
        cv_canny = cv2.GaussianBlur(cv_canny, (3, 3), 0.6)
        cv_canny = cv2.Canny(cv_canny, lower, upper)
        cv_canny = cv2.dilate(cv_canny, (5, 5), iterations=1)
        contours, hierarchy = cv2.findContours(cv_canny,
                      cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        return contours

    # from list of contours find the closest 'blob' to a point
    def find_blob(self, target_pt, contours):
        closest_contour = []
        shortest_distance = None
        target_centroid = []

        for c in contours:
            M = cv2.moments(c)
            if M['m00'] == 0.0:
                continue

            cX = M["m10"] / M["m00"]
            cY = M["m01"] / M["m00"]

            centroid = np.array((cX, cY, 0.0))

            distance = np.linalg.norm(target_pt - centroid)
            if len(closest_contour) == 0: 
                closest_contour = c
                shortest_distance = distance
                target_centroid = centroid
            else:
                if distance < shortest_distance:
                    closest_contour = c
                    shortest_distance = distance
        
        return closest_contour, target_centroid


def main():
    rclpy.init()

    sub = ARCamera()
    rclpy.spin(sub)

    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
