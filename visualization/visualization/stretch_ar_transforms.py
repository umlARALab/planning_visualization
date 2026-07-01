import rclpy
from rclpy.node import Node

from stretch_ar.msg import HitPos
from geometry_msgs.msg import Transform, PointStamped

import numpy as np
from scipy.spatial.transform import Rotation as R


class ARTransform(Node):
    def __init__(self):
        super().__init__('ar_transform')

        self.unity_tf_stretch = np.array([])
        # self.world_pt_stretch_history = []

        self.unity_tf = Transform()

        # subscribe to stretch align point published by quest
        self.aruco_marker_sub = self.create_subscription(
            HitPos,
            '/quest_stretch_align',
            self.stretch_align,
            10
        )

        # subscribe to object hit point published by quest
        self.aruco_marker_sub = self.create_subscription(
            HitPos,
            '/quest_hit_point',
            self.hit_point_position,
            10
        )

        # publish object position relative to stretch
        self.obj_point_pub = self.create_publisher(PointStamped, '/object_position', 10)
        self.stretch_tf_unity_pub = self.create_publisher(Transform, '/unity_tf', 10)

    def stretch_align(self, msg):
        if msg == None:
            return

        # get pose of stretch based off of drawn line in unity in order to transform to unity world frame        
        stretch_v = [
            msg.end_position.x - msg.hit_position.x, 
            msg.end_position.y - msg.hit_position.y, 
            msg.end_position.z - msg.hit_position.z
        ]
        unity_v = [1, 0, 0]

        # get tf from stretch -> unity
        unity_rot_stretch = self.get_rotation_matrix(unity_v, stretch_v)
        self.unity_tf_stretch = np.array([
            [unity_rot_stretch[0][0], unity_rot_stretch[0][1], unity_rot_stretch[0][2], msg.hit_position.x],
            [unity_rot_stretch[1][0], unity_rot_stretch[1][1], unity_rot_stretch[1][2], msg.hit_position.y],
            [unity_rot_stretch[2][0], unity_rot_stretch[2][1], unity_rot_stretch[2][2], 0],
            [0, 0, 0, 1]
        ])

        # publish tf stretch tf unity to keep track of
        stretch_tf_unity = np.linalg.inv(self.unity_tf_stretch)
        self.unity_tf.translation = [stretch_tf_unity[0][4], stretch_tf_unity[1][4], stretch_tf_unity[2][4]]
        unity_rot = R.from_matrix(np.array(
            [stretch_tf_unity[0][0], stretch_tf_unity[0][1], stretch_tf_unity[0][2]],
            [stretch_tf_unity[1][0], stretch_tf_unity[1][1], stretch_tf_unity[1][2]],
            [stretch_tf_unity[2][0], stretch_tf_unity[2][1], stretch_tf_unity[2][2]],        
        ))
        self.unity_tf.rotation = unity_rot.as_quat()

        self.stretch_tf_unity_pub.publish(self.unity_tf)

        print('UNITY -> STRETCH')
        print(self.unity_tf_stretch)
        print('')

    def hit_point_position(self, msg):
        # init publisher stuff
        obj_pt = PointStamped()
        obj_pt.header.frame_id = "base_link"
        obj_pt.header.stamp = self.get_clock().now().to_msg()

        # obj to unity -> unity to stretch
        obj_tf_unity = np.array([
            [1, 0, 0, -msg.hit_position.x],
            [0, 1, 0, -msg.hit_position.y],
            [0, 0, 1, -msg.hit_position.z],
            [0, 0, 0, 1]
        ])

        obj_tf_stretch = obj_tf_unity @ self.unity_tf_stretch
        stretch_tf_obj = np.linalg.inv(obj_tf_stretch)
        print(stretch_tf_obj)

        obj_pt.point.x = stretch_tf_obj[0][3]
        obj_pt.point.y = stretch_tf_obj[1][3]
        obj_pt.point.z = stretch_tf_obj[2][3]

        self.obj_point_pub.publish(obj_pt)

    # get rotation matrix to align v1 with v2
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

    sub = ARTransform()
    rclpy.spin(sub)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
