import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PointStamped, PoseStamped
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import numpy as np
from scipy.spatial.transform import Rotation as R

base_frame = 'base_link' 
camera_frame = 'oak_rgb_camera_optical_frame'

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
        super().__init__('aruco_camera_position')
        
        # subscribers to aruco marker topic and the color
        self.aruco_marker_sub = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.aruco_callback,
            10
        )
        # self.camera_sub = self.create_subscription(
        #     Image,
        #     '/oak/rgb/image_raw',
        #     self.cam_callback(),
        #     10
        # )
        self.cam_pose_pub = self.create_publisher(PoseStamped, '/camera_position', 10)
        self.marker_pub = self.create_publisher(PointStamped, '/marker_position', 10)


    def aruco_callback(self, msg):
        i = 0
        cam_pose = PoseStamped()
        # pt = PointStamped()
        marker = PointStamped()

        # cam_pose.header.frame_id = camera_frame # camera frame
        marker.header.frame_id = base_frame 
        # cam_pose.header.stamp = self.get_clock().now().to_msg()
        marker.header.stamp = self.get_clock().now().to_msg()

        pose_list = []

        tf_baselink_to_cam = np.array([])
        tf_cam_to_baselink = np.array([])
        # print('marker pose : \n' + str(msg.poses[0]) + '\n')

        # check for pose detection
        # if msg.marker_ids == 131 or msg.marker_ids == 130:
        # check for correct position
        # if msg.marker_ids == 131:
        #     marker.point = msg.poses[0].position
            # print('marker : \n' + str(marker.point) + '\n')

        for id in msg.marker_ids:
            # position = msg.poses[i].position
            # orientation = msg.poses[i].orientation
            # pose = PoseStamped()

            # rot = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
            # rot_mat = rot.as_matrix()

            # # transform: cam --> aruco (base) markers
            # tf_cam_to_marker = np.array([
            #     [rot_mat[0][0], rot_mat[0][1], rot_mat[0][2], position.x],
            #     [rot_mat[1][0], rot_mat[1][1], rot_mat[1][2], position.y],
            #     [rot_mat[2][0], rot_mat[2][1], rot_mat[2][2], position.z],
            #     [0.0, 0.0, 0.0, 1.0]
            # ])

            # print('cam to marker :\n' + str(tf_cam_to_marker))

            # transform: cam --> base (known fixed transform from tf)
            # if id == 130:   # base_left
            #     tf_cam_to_baselink = tf_cam_to_marker @ tf_base_left_to_base_link
            #     # tf_baselink_to_cam = tf_cam_to_marker * tf_base_left_to_base_link

            #     marker.point.x = tf_cam_to_baselink[0][3]
            #     marker.point.y = tf_cam_to_baselink[1][3]
            #     marker.point.z = tf_cam_to_baselink[2][3]

            #     # print('cam to base:\n' + str(tf_cam_to_baselink))
            # el
            if id == 131:   # base_right
                position = msg.poses[i].position
                orientation = msg.poses[i].orientation
                pose = PoseStamped()

                rot = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
                rot_mat = rot.as_matrix()

                # transform: cam --> aruco (base) markers
                tf_cam_to_marker = np.array([
                    [rot_mat[0][0], rot_mat[0][1], rot_mat[0][2], position.x],
                    [rot_mat[1][0], rot_mat[1][1], rot_mat[1][2], position.y],
                    [rot_mat[2][0], rot_mat[2][1], rot_mat[2][2], position.z],
                    [0.0, 0.0, 0.0, 1.0]
                ])

                tf_cam_to_baselink = tf_cam_to_marker @ tf_base_right_to_base_link
                # tf_baselink_to_cam = tf_cam_to_marker * tf_base_right_to_base_link
                
                marker.point.x = tf_cam_to_baselink[0][3]
                marker.point.y = tf_cam_to_baselink[1][3]
                marker.point.z = tf_cam_to_baselink[2][3]

                # print('base from right: ' + str(marker.point))

                # transform baselink -> cam == inverse(cam --> baselink)
                tf_baselink_to_cam = np.linalg.inv(tf_cam_to_baselink)

                self.marker_pub.publish(marker)

                pose.pose.position.x = tf_baselink_to_cam[0][3] # + marker.point.x
                pose.pose.position.y = tf_baselink_to_cam[1][3] # + marker.point.y
                pose.pose.position.z = tf_baselink_to_cam[2][3] # + marker.point.z

                new_rot = R.from_matrix([
                    [tf_baselink_to_cam[0][0], tf_baselink_to_cam[0][1], tf_baselink_to_cam[0][2]],
                    [tf_baselink_to_cam[1][0], tf_baselink_to_cam[1][1], tf_baselink_to_cam[1][2]],
                    [tf_baselink_to_cam[2][0], tf_baselink_to_cam[2][1], tf_baselink_to_cam[2][2]]
                ])

                # pose.orientation = orientation
                pose.pose.orientation.x = (new_rot.as_quat())[0]
                pose.pose.orientation.y = (new_rot.as_quat())[1]
                pose.pose.orientation.z = (new_rot.as_quat())[2]
                pose.pose.orientation.w = (new_rot.as_quat())[3]

                cam_pose.pose = pose.pose

                cam_pose.header.frame_id = base_frame
                cam_pose.header.stamp = self.get_clock().now().to_msg()
                # if pose_list:
                    # cam_pose.poses = pose_list
                    # print('left pose : \n' + str(pose_list[0]) + '\n')
                    # print('right pose : \n' + str(pose_list[1]) + '\n')
                
                # print(cam_pose)
                self.cam_pose_pub.publish(cam_pose)
            else:
                continue

            # transform baselink -> cam == inverse(cam --> baselink)
            # tf_baselink_to_cam = np.linalg.inv(tf_cam_to_baselink)

            # self.marker_pub.publish(marker)

            # pose.pose.position.x = tf_baselink_to_cam[0][3] # + marker.point.x
            # pose.pose.position.y = tf_baselink_to_cam[1][3] # + marker.point.y
            # pose.pose.position.z = tf_baselink_to_cam[2][3] # + marker.point.z

            # new_rot = R.from_matrix([
            #     [tf_baselink_to_cam[0][0], tf_baselink_to_cam[0][1], tf_baselink_to_cam[0][2]],
            #     [tf_baselink_to_cam[1][0], tf_baselink_to_cam[1][1], tf_baselink_to_cam[1][2]],
            #     [tf_baselink_to_cam[2][0], tf_baselink_to_cam[2][1], tf_baselink_to_cam[2][2]]
            # ])

            # # pose.orientation = orientation
            # pose.pose.orientation.x = (new_rot.as_quat())[0]
            # pose.pose.orientation.y = (new_rot.as_quat())[1]
            # pose.pose.orientation.z = (new_rot.as_quat())[2]
            # pose.pose.orientation.w = (new_rot.as_quat())[3]

            # print('pose :\n' + str(tf_baselink_to_cam))

            # print(str(id) + ': \n' + str(pose) + '\n')pose_list

            # pose.pose.orientation.x = orientation.x
            # pose.pose.orientation.y = orientation.y
            # pose.pose.orientation.z = orientation.z
            # pose.pose.orientation.w = orientation.w

            # pose_list.append(pose)
        
        # avg_pos = [0.0, 0.0, 0.0]
        # for pt in pose_list:
        #     avg_pos[0] += pt.pose.position.x
        #     avg_pos[1] += pt.pose.position.y
        #     avg_pos[2] += pt.pose.position.z

        # if pose_list:
        #     cam_pose = pose_list[0] # get orientation
        #     cam_pose.pose.position.x = avg_pos[0] / len(pose_list)
        #     cam_pose.pose.position.y = avg_pos[1] / len(pose_list)
        #     cam_pose.pose.position.z = avg_pos[2] / len(pose_list)
        # cam_pose.pose = pose.pose

        # cam_pose.header.frame_id = base_frame
        # cam_pose.header.stamp = self.get_clock().now().to_msg()
        # # if pose_list:
        #     # cam_pose.poses = pose_list
        #     # print('left pose : \n' + str(pose_list[0]) + '\n')
        #     # print('right pose : \n' + str(pose_list[1]) + '\n')
        
        # # print(cam_pose)
        # self.cam_pose_pub.publish(cam_pose)


def main():
    rclpy.init()

    sub = ArucoDetect()
    rclpy.spin(sub)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
