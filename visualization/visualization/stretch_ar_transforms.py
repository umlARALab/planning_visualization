import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Pose
import numpy as np
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R


from stretch_ar.msg import HitPos

class ARTransform(Node):
    def __init__(self):
        super().__init__('ar_transform')

        self.unity_tf_world = np.array([])
        # self.world_pt_stretch_history = []
        self.unity_tf_stretch = np.array([])
        self.stretch_pose = Pose()
        
        self.stretch_pose.position.x = 0.0
        self.stretch_pose.position.y = 0.0
        self.stretch_pose.position.z = 0.0
        self.stretch_pose.orientation.x = 0.0
        self.stretch_pose.orientation.y = 0.0
        self.stretch_pose.orientation.z = 0.0
        self.stretch_pose.orientation.w = 1.0

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

        self.joint_sub = self.create_subscription(
            Pose,
            '/stretch_odom', # get stretch odom topic
            self.odom_callback,
            10
        )

        # publish object position relative to stretch
        self.obj_point_pub = self.create_publisher(PointStamped, '/object_position', 10)

    # get current stretch position to turn towards object
    def odom_callback(self, msg):
        # update current position relative to world
        self.stretch_pose.position.x += msg.position.x
        self.stretch_pose.position.y += msg.position.y
        self.stretch_pose.position.z = 0.0

        og_rot = R.from_quat([
            self.stretch_pose.orientation.x,
            self.stretch_pose.orientation.y,
            self.stretch_pose.orientation.z,
            self.stretch_pose.orientation.w
        ])

        msg_rot = R.from_quat([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])

        new_rot = R.from_euler('xyz', [0, 0, og_rot.as_euler('xyz')[2] + msg_rot.as_euler('xyz')[2]])
        self.stretch_pose.orientation.x = new_rot.as_quat()[0]
        self.stretch_pose.orientation.y = new_rot.as_quat()[1]
        self.stretch_pose.orientation.z = new_rot.as_quat()[2]
        self.stretch_pose.orientation.w = new_rot.as_quat()[3]

        print('old angle: ' + str(og_rot.as_euler('xyz', True)[2]) + ' | new angle: ' + str(msg_rot.as_euler('xyz', True)[2]))

        print((new_rot.as_euler('xyz') * 180) / np.pi)

        world_rot_stretch = new_rot.as_matrix()

        world_tf_stretch = np.array([
            [world_rot_stretch[0][0], world_rot_stretch[0][1], world_rot_stretch[0][2], self.stretch_pose.position.x],
            [world_rot_stretch[1][0], world_rot_stretch[1][1], world_rot_stretch[1][2], self.stretch_pose.position.y],
            [world_rot_stretch[2][0], world_rot_stretch[2][1], world_rot_stretch[2][2], 0],
            [0, 0, 0, 1]
        ])

        if self.unity_tf_world.size != 0:
            self.unity_tf_stretch = self.unity_tf_world @ world_tf_stretch


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
        stretch_rot_unity = self.get_rotation_matrix(unity_v, stretch_v)
        self.unity_tf_stretch = np.array([
            [stretch_rot_unity[0][0], stretch_rot_unity[0][1], stretch_rot_unity[0][2], msg.hit_position.x],
            [stretch_rot_unity[1][0], stretch_rot_unity[1][1], stretch_rot_unity[1][2], msg.hit_position.y],
            [stretch_rot_unity[2][0], stretch_rot_unity[2][1], stretch_rot_unity[2][2], 0],
            [0, 0, 0, 1]
        ])
        if self.unity_tf_world.size == 0:
            self.unity_tf_world = self.unity_tf_stretch

        # print('UNITY -> STRETCH WORLD')
        # print(self.unity_tf_world)
        # print('')

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
        # print(stretch_tf_obj)

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
