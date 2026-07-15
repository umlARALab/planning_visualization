import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

from stretch_ar.msg import HitPos

class ARCamera(Node):
    def __init__(self):
        super().__init__('ar_camera')

        self.bridge = CvBridge()

        self.aruco_marker_sub = self.create_subscription(
            Image,
            '/quest_camera',
            self.quest_cam_callback,
            10
        )

    def quest_cam_callback(self, img):
        cv_image = self.bridge.imgmsg_to_cv2(img, 'rgb8')

        cv2.imshow(cv_image)
        cv2.waitKey(1)

    

def main():
    rclpy.init()

    sub = ARCamera()
    rclpy.spin(sub)

    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
