import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ARDetect(Node):
    def __init__(self):
        super().__init__('ar_detect')

        self.bridge = CvBridge()

        self.quest_cam = self.create_subscription(
            Image,
            '/quest_camera',
            self.quest_callback,
            10
        )

    def quest_callback(self, rosImg):
        # test quest image publish
        self.image = self.bridge.imgmsg_to_cv2(rosImg, "rgb8")

        cv2.imshow(self.image)
        cv2.waitKey(1)

def main():
    rclpy.init()

    sub = ARDetect()
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
