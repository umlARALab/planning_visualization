import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PointStamped, PoseStamped, Pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from stretch_ar.msg import RobotStatus

import numpy as np
from scipy.spatial.transform import Rotation as R

# GIVEN A POINT, PLAN A TRAJECTORY TO THAT POINT AND PUBLISH A LIST OF WAYPOINTS

class ARPlanTrajectory(Node):
    def __init__(self):
        super().__init__('ar_trajectory')

        self.obj_estimation_sub = self.create_subscription(
            PointStamped,
            '/object_position',
            self.locate_callback,
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

def main():
    rclpy.init()

    sub = ARPlanTrajectory()
    rclpy.spin(sub)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
