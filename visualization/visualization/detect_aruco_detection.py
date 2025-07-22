import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PointStamped, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import numpy as np
from scipy.spatial.transform import Rotation as R

stretch_aruco_ids = {
    'base_left': {
        'id': 130,
        'link': 'link_aruco_left_base'
    },
    'base_right': {
        'id': 131,
        'link': 'link_aruco_right_base'
    },
    'wrist_inside': {
        'id': 132,
        'link': 'link_aruco_inner_wrist'
    },
    'wrist_top': {
        'id': 133, 
        'link': 'link_aruco_top_wrist'
    },
    'shoulder_top': {
        'id': 134,
        'link': 'link_aruco_shoulder'
    }
}

# transform information
tf_base_left_to_base_link = np.array([
    [-0.0, -1.0,  0.0,  0.130],
    [1.0, -0.0,  0.0,  0.005],
    [0.0,  0.0,  1.0, -0.160],
    [0.0,  0.0,  0.0,  1.0]
])


tf_base_right_to_base_link = np.array([
    [-0.0, -1.0,  0.0, -0.130],
    [1.0, -0.0,  0.0,  0.002],
    [0.0,  0.0,  1.0, -0.160],
    [0.0,  0.0,  0.0,  1.0]
])

class ArucoDetect(Node):
    def __init__(self):
        super().__init__('aruco_detection')
        
        # subscribers to aruco marker topic and the color
        self.aruco_marker_sub = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.aruco_callback,
            10
        )
        # self.camera_sub = self.create_subscription(
        #     Image,
        #     '/camera/color/image_raw',
        #     self.cam_callback(),
        #     10
        # )
        self.cam_pose_pub = self.create_publisher(PoseArray, '/camera_position', 10)
        self.marker_pub = self.create_publisher(PointStamped, '/marker_position', 10)


    def aruco_callback(self, msg):
        i = 0
        cam_pose = PoseArray()
        # pt = PointStamped()
        marker = PointStamped()
        cam_pose.header.frame_id = 'camera_color_optical_frame' # camera frame
        marker.header.frame_id = 'camera_color_optical_frame' # camera frame
        cam_pose.header.stamp = self.get_clock().now().to_msg()
        marker.header.stamp = self.get_clock().now().to_msg()

        pose_list = []

        tf_baselink_to_cam = np.array([])
        # print('marker pose : \n' + str(msg.poses[0]) + '\n')

        marker.point = msg.poses[0].position
        self.marker_pub.publish(marker)
        # print('marker : \n' + str(marker.point) + '\n')
        
        for id in msg.marker_ids:
            position = msg.poses[i].position
            orientation = msg.poses[i].orientation
            pose = Pose()

            rot = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
            rot_mat = rot.as_matrix()

            tf_cam_to_marker = np.array([
                [rot_mat[0][0], rot_mat[0][1], rot_mat[0][2], position.x],
                [rot_mat[1][0], rot_mat[1][1], rot_mat[1][2], position.y],
                [rot_mat[2][0], rot_mat[2][1], rot_mat[2][2], position.z],
                [0.0, 0.0, 0.0, 1.0]
            ])

            # transform from world to camera
            if id == 130:   # base_left
                # tf_baselink_to_cam = np.linalg.inv(tf_cam_to_marker * tf_base_left_to_base_link)
                tf_baselink_to_cam = tf_cam_to_marker * tf_base_left_to_base_link
            if id == 131:   # base_right
                tf_baselink_to_cam = np.linalg.inv(tf_cam_to_marker * tf_base_right_to_base_link)

            pose.position.x = tf_baselink_to_cam[0][3]
            pose.position.y = tf_baselink_to_cam[1][3]
            pose.position.z = tf_baselink_to_cam[2][3]

            pose.orientation.x = (rot.as_quat())[0]
            pose.orientation.y = (rot.as_quat())[1]
            pose.orientation.z = (rot.as_quat())[2]
            pose.orientation.w = (rot.as_quat())[3]

            # print(str(id) + ': \n' + str(pose) + '\n')

            pose_list.append(pose)

        if pose_list:
            cam_pose.poses = pose_list
            print('left pose : \n' + str(pose_list[0]) + '\n')
            # print('right pose : \n' + str(pose_list[1]) + '\n')

            self.cam_pose_pub.publish(cam_pose)


def main():
    rclpy.init()

    sub = ArucoDetect()
    rclpy.spin(sub)

    rclpy.shutdown()

if __name__ == '__main__':
    main()