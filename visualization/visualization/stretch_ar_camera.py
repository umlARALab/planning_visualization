import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
import numpy as np

from stretch_ar.msg import ImageTarget
from geometry_msgs.msg import Vector3

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

        # self.target_image = cv2.Mat()

    def quest_cam_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg.image, 'bgr8')
        # cv2.circle(cv_image, (int(msg.position.x), int(msg.position.y)), 5, color=(0, 255, 0), thickness=-1)
        screen_target = np.array((msg.position.x, msg.position.y, msg.position.z))
        upper = 120
        lower = 40

        # get canny image to get outlines
        cv_canny = cv2.addWeighted(cv_image, 1.5, np.zeros(cv_image.shape, cv_image.dtype), 0, 0)
        cv_canny = cv2.GaussianBlur(cv_canny, (3, 3), 0.6)
        cv_canny = cv2.Canny(cv_canny, lower, upper)
        cv_canny = cv2.dilate(cv_canny, (5, 5), iterations=1)
        contours, hierarchy = cv2.findContours(cv_canny,
                      cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        # go through edges and find target
        closest_contour = []
        shortest_distance = None
        for c in contours:
            M = cv2.moments(c)
            if M['m00'] == 0.0:
                continue

            cX = M["m10"] / M["m00"]
            cY = M["m01"] / M["m00"]

            centroid = np.array((cX, cY, 0.0))

            distance = np.linalg.norm(screen_target - centroid)

            if len(closest_contour) == 0: 
                closest_contour = c
                shortest_distance = distance
            else:
                if distance < shortest_distance:
                    closest_contour = c
                    shortest_distance = distance

        copy_image = cv_image.copy()

        cv2.drawContours(copy_image, contours, -1, (0, 255, 0), 1)
        cv2.drawContours(copy_image, closest_contour, -1, (0, 0, 255), 3)
        box = cv2.boundingRect(closest_contour)
        x, y, w, h = box
        w_padding = int(w *0.25)
        h_padding = int(h * 0.25)
        x -= w_padding
        y -= h_padding

        image_scale = int(240 / (h + h_padding))

        target_image = cv_image[y:(y+h+(h_padding*2)), x:(x+w+(w_padding*2))]
        self.target_image = cv2.resize(target_image, None, fx=image_scale, fy=image_scale, interpolation=cv2.INTER_LINEAR)
                
        cv2.imshow('Contours', copy_image)
        cv2.imshow('Target', self.target_image)

        cv2.waitKey(1)

def main():
    rclpy.init()

    sub = ARCamera()
    rclpy.spin(sub)

    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
