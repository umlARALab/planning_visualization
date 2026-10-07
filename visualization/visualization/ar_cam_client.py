import sys

from stretch_ar.srv import ARCam
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from stretch_ar.msg import ImageTarget, RobotStatus
import numpy as np
from cv_bridge import CvBridge
import cv2

class camera_client(Node):

    def __init__(self):
        super().__init__('camera_client')

        self.bridge = CvBridge()
        self.target_image = None
        self.camera_image = None

        # camera subscribers
        self.robot_status = self.create_subscription(
            RobotStatus,
            'robot_feedback',
            self.status_callback,
            10
        )
        self.aruco_marker_sub = self.create_subscription(
            ImageTarget,
            '/quest_camera',
            self.quest_cam_callback,
            10
        )
        self.stretch_camera_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            10
        )

        self.state = 0
        self.status_pub = self.create_publisher(RobotStatus, '/robot_feedback', 10)

        # service client
        self.cli = self.create_client(ARCam, 'get_target_pos')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ARCam.Request()

        
    def quest_cam_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='rgb8')
        copy_image = cv_image.copy()
        screen_target = np.array((msg.position.x, msg.position.y, msg.position.z))

        contours = self.get_contours(cv_image)

        # go through edges and find target
        closest_contour, centroid = self.find_blob(screen_target, contours)

        # copy image shows contours for debugging and testing purposes
        cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 1)
        cv2.drawContours(cv_image, closest_contour, -1, (0, 0, 255), 3)

        # crop the potential target object 
        box = cv2.boundingRect(closest_contour)
        x, y, w, h = box
        w_padding = int(w * 0.25)
        h_padding = int(h * 0.25)
        x -= w_padding
        y -= h_padding
        image_scale = 1

        # scale cropped image to make it more viewable
        # if h > w:
        #     image_scale = int(240 / (h + h_padding))
        # else:
        #     image_scale = int(240 / (w + w_padding))

        # target = cv_image[y:(y+h+(h_padding*2)), x:(x+w+(w_padding*2))]
        # print(target.shape)
        # self.target_image = self.bridge.cv2_to_imgmsg(target, encoding='bgr8')
        self.target_image = msg.image
        # big_target_image = cv2.resize(target, None, fx=image_scale, fy=image_scale, interpolation=cv2.INTER_LINEAR)
                
        # cv2.imshow('Contours', copy_image)
        cv2.imshow('Target', cv_image)
        cv2.waitKey(1)


    def camera_callback(self, msg):
        # ensure image exists
        if self.state == 2:
            self.camera_image = msg


    def status_callback(self, msg):
        self.state = msg.state
        self.get_logger().info(f'State: {self.state}')
        if msg.state == 3:
            if self.camera_image == None or self.target_image == None:
                self.get_logger().info('Select object again')
                if self.camera_image == None:
                    self.get_logger().info('no cam image')
                else:
                    self.get_logger().info('no target image')

                update_state = RobotStatus()
                update_state.state = 2
                update_state.data = 'Reselect target'

                self.status_pub.publish(update_state)
            else:
                response = self.send_request(self.camera_image, self.target_image)

                cv_sift = self.bridge.imgmsg_to_cv2(response.sift_match)
                cv2.imshow('SIFT', cv_sift)
                cv2.waitKey(1)


    def send_request(self, cam_img, tar_img):
        self.req.camera_view = cam_img
        self.req.img_target = tar_img

        self.get_logger().info('Client sent request')
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('Service done')

        return self.future.result()

    # ---------- HELPER FUNCTIONS -----------
    def get_contours(self, img):
        upper = 120
        lower = 40

        # get canny image to get outlines
        cv_canny = cv2.addWeighted(img, 1.5, np.zeros(img.shape, img.dtype), 0, 0)
        cv_canny = cv2.GaussianBlur(cv_canny, (3, 3), 0.6)
        cv_canny = cv2.Canny(cv_canny, lower, upper)
        cv_canny = cv2.dilate(cv_canny, (5, 5), iterations=1)
        contours, hierarchy = cv2.findContours(cv_canny,
                      cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        return contours

    # from list of contours find the closest 'blob' to a point
    def find_blob(self, target_pt, contours):
        closest_contour = []
        shortest_distance = None
        target_centroid = []

        for c in contours:
            M = cv2.moments(c)
            if M['m00'] == 0.0:
                continue

            cX = M["m10"] / M["m00"]
            cY = M["m01"] / M["m00"]

            centroid = np.array((cX, cY, 0.0))

            distance = np.linalg.norm(target_pt - centroid)
            if len(closest_contour) == 0: 
                closest_contour = c
                shortest_distance = distance
                target_centroid = centroid
            else:
                if distance < shortest_distance:
                    closest_contour = c
                    shortest_distance = distance
        
        return closest_contour, target_centroid


def main(args=None):
    rclpy.init(args=args)

    minimal_client = camera_client()
    rclpy.spin(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()