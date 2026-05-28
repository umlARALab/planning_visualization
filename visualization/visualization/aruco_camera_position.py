import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PointStamped, PoseStamped, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from ros2_numpy import numpify
import numpy as np
from scipy.spatial.transform import Rotation as R

base_frame = 'base_link' 
camera_frame = 'oak_rgb_camera_optical_frame'

stretch_aruco_ids = {
    130: 'link_aruco_left_base',
    131: 'link_aruco_right_base', 
    132: 'link_aruco_inner_wrist', 
    133: 'link_aruco_top_wrist',
    134: 'link_aruco_shoulder',
    135: 'link_aruco_d405',
    200: 'link_aruco_fingertip_left',
    201: 'link_aruco_fingertip_right'
}

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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cam_pose_pub = self.create_publisher(PoseStamped, '/camera_position', 10)

    def get_marker_tf(self, marker_id):
        marker_tf_baselink_mat = []

        if marker_id in stretch_aruco_ids:
            try:
                marker_tf_baselink = self.tf_buffer.lookup_transform(
                    stretch_aruco_ids[marker_id],
                    base_frame,
                    rclpy.time.Time()
                )
                marker_tf_baselink_mat = numpify(marker_tf_baselink.transform)
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {stretch_aruco_ids[marker_id]} to {base_frame}: {ex}')
        else: 
            print(marker_id)
        
        return marker_tf_baselink_mat

    def aruco_callback(self, msg):
        cam_pose = PoseStamped()
        # pt = PointStamped()

        cam_pose.header.frame_id = base_frame
        cam_pose.header.stamp = self.get_clock().now().to_msg()

        pose_list = []

        tf_baselink_to_cam = np.array([])
        tf_cam_to_baselink = np.array([])


        # base_tfs = self.get_base_tf()

        # use i to match id and pose index
        for i in range(len(msg.marker_ids)):
            id = msg.marker_ids[i]
            marker_pose = msg.poses[i]
            pose = Pose()

            rot = R.from_quat([marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w])
            rot_mat = rot.as_matrix()

            # # transform: cam --> aruco marker
            tf_cam_to_marker = np.array([
                [rot_mat[0][0], rot_mat[0][1], rot_mat[0][2], marker_pose.position.x],
                [rot_mat[1][0], rot_mat[1][1], rot_mat[1][2], marker_pose.position.y],
                [rot_mat[2][0], rot_mat[2][1], rot_mat[2][2], marker_pose.position.z],
                [0.0, 0.0, 0.0, 1.0]
            ])

            # get tf marker to baselink
            tf_marker_to_baselink = self.get_marker_tf(id)

            tf_cam_to_baselink = tf_cam_to_marker @ tf_marker_to_baselink
            tf_baselink_to_cam = np.linalg.inv(tf_cam_to_baselink)

            pose.position.x = tf_baselink_to_cam[0][3]
            pose.position.y = tf_baselink_to_cam[1][3]
            pose.position.z = tf_baselink_to_cam[2][3]

            baselink_to_cam_rot = np.array([
                [tf_baselink_to_cam[0][0], tf_baselink_to_cam[0][1], tf_baselink_to_cam[0][2]],
                [tf_baselink_to_cam[1][0], tf_baselink_to_cam[1][1], tf_baselink_to_cam[1][2]],
                [tf_baselink_to_cam[2][0], tf_baselink_to_cam[2][1], tf_baselink_to_cam[2][2]]
            ])

            # correct camera frame rotation matrix
            cam_to_world_rot = np.array([
                [0, -1.0, 0],
                [0, 0, -1.0],
                [1.0, 0, 0]
            ])

            new_rot = R.from_matrix(baselink_to_cam_rot @ cam_to_world_rot)

            pose.orientation.x = (new_rot.as_quat())[0]
            pose.orientation.y = (new_rot.as_quat())[1]
            pose.orientation.z = (new_rot.as_quat())[2]
            pose.orientation.w = (new_rot.as_quat())[3]

            pose_list.append(pose)
            
        # average camera poses from all aruco pose estimations
        total = Pose()
        for pose in pose_list:
            total.position.x += pose.position.x
            total.position.y += pose.position.y
            total.position.z += pose.position.z

            total.orientation.x += pose.orientation.x
            total.orientation.y += pose.orientation.y
            total.orientation.z += pose.orientation.z
            total.orientation.w += pose.orientation.w

        if pose_list:
            cam_pose.pose.position.x = total.position.x / len(pose_list)
            cam_pose.pose.position.y = total.position.y / len(pose_list)
            cam_pose.pose.position.z = total.position.z / len(pose_list)
            cam_pose.pose.orientation.x = total.orientation.x / len(pose_list)
            cam_pose.pose.orientation.y = total.orientation.y / len(pose_list)
            cam_pose.pose.orientation.z = total.orientation.z / len(pose_list)
            cam_pose.pose.orientation.w = total.orientation.w / len(pose_list)

            # published calculated average camera position 
            self.cam_pose_pub.publish(cam_pose)
        

def main():
    rclpy.init()

    sub = ArucoDetect()
    rclpy.spin(sub)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
