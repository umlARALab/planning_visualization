import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Pose
from nav_msgs.msg import Odometry

import numpy as np

class LocateTarget(Node):
    def __init__(self):
        super().__init__('locate_object')

        self.stretch_pose = Pose()

        self.joint_sub = self.create_subscription(
            Odometry,
            '/odometry', # get stretch odom topic
            self.odom_callback,
            10
        )

        # get rough position estimation of target object
        self.obj_estimation_sub = self.create_subscription(
            PointStamped,
            '/object_position',
            self.locate_callback,
            10
        )

    # get current stretch position to turn towards object
    def odom_callback(self, msg):
        self.stretch_pose = msg.pose.pose

    # turn stretch camera to look at target object position
    def locate_callback(self, msg):
        obj_pt = msg.point

        obj_v = [obj_pt.x, obj_pt.y, 0]
        stretch_v = [1, 0, 0]

        # measure rotation from stretch to target object
        stretch_rot_obj = self.get_rotation_matrix(stretch_v, obj_v)

        

    def get_rotation_matrix(self, v1, v2):

        a = (v1 / np.linalg.norm(v1)).reshape(3)
        b = (v2 / np.linalg.norm(v2)).reshape(3)

        v = np.cross(a, b)
        c = np.dot(a, b)
        s = np.linalg.norm(v)

        kmat = np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])

        rot_mat = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
        return rot_mat






# def main():
#     rclpy.init()

#     sub = ARTransform()
#     rclpy.spin(sub)

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
