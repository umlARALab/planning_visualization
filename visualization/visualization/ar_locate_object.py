import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Pose, Twist
from nav_msgs.msg import Odometry

import numpy as np
from scipy.spatial.transform import Rotation as R
from tf_transformations import euler_from_quaternion


class LocateTarget(Node):
    def __init__(self):
        super().__init__('locate_object')

        self.stretch_pose = Pose()

        self.target_rotation = []
        self.target_input = False

        self.joint_sub = self.create_subscription(
            Odometry,
            '/odom', # get stretch odom topic
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

        self.cmd_pub = self.create_publisher(
            Twist, 
            '/stretch/cmd_vel',
            10
        )

    # get current stretch position to turn towards object
    def odom_callback(self, msg):
        self.stretch_pose = msg.pose.pose
        move = Twist()

        if self.target_input:
            if self.compare_rotation() == -1:
                move.angular.z = -0.1
                self.cmd_pub.publish(move)
            elif self.compare_rotation() == 1:
                move.angular.z = 0.1
                self.cmd_pub.publish(move)
            else:
                move.angular.z = 0.0
                self.cmd_pub.publish(move)


    def compare_rotation(self):
        stretch_rot = euler_from_quaternion([self.stretch_pose.orientation.x,
                                              self.stretch_pose.orientation.y,
                                              self.stretch_pose.orientation.z,
                                              self.stretch_pose.orientation.w])

        if np.abs(stretch_rot[2] - self.target_rotation[2]) < 0.1:
            print('reached target angle')
            self.target_input = False
            return 0 # don't move
        elif self.target_rotation[2] > 0:
            return 1 # turn left (counter cw)
        else :
            return -1 # turn right (clockwise)


    # turn stretch camera to look at target object position
    def locate_callback(self, msg):
        obj_pt = msg.point
        self.target_input = True

        obj_v = [obj_pt.x, obj_pt.y, 0]
        stretch_v = [1, 0, 0]

        # measure rotation from stretch to target object
        stretch_rot_obj = R.from_matrix(self.get_rotation_matrix(stretch_v, obj_v))

        self.target_rotation = stretch_rot_obj.as_euler('xyz')
        print(self.target_rotation)


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

def main():
    rclpy.init()

    sub = LocateTarget()
    rclpy.spin(sub)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


