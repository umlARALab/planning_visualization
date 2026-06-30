import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
import numpy as np

import stretch_body.robot

class LocateTarget(Node):
    def __init__(self):
        super().__init__('locate_object')

        self.obj_estimation_sub = self.create_subscription(
            PointStamped,
            '/object_position',
            self.locate_callback,
            10
        )

    def locate_callback(self, msg):
        


# def main():
#     rclpy.init()

#     sub = ARTransform()
#     rclpy.spin(sub)

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
