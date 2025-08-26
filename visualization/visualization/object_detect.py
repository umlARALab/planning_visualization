import time

import ros2_numpy
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
import cv2

from ultralytics import YOLO

model = "best.pt"
# cam_topic = '/oak/rgb/image_raw'
cam_topic = '/camera/color/image_raw'

# 1280x720 ? for high res

class ObjectDetection(Node):
    def __init__(self):
        self.detection_model = YOLO(model)
        super().__init__('ultralytics')
        time.sleep(1)

        self.subscription = self.create_subscription(Image, cam_topic, self.callback, 10)
        self.det_image_pub = self.create_publisher(Image, '/ultralytics/detection/image', 10)
        self.obj_pos_pub = self.create_publisher(PointStamped, '/object_detection_position')

    def callback(self, data):
        """Callback function to process image and publish annotated images."""
        array = ros2_numpy.numpify(data)
        print('height: ' + str(data.height))
        print('width: ' + str(data.width))
        obj_pos_arr = np.array()

        pt = PointStamped()
        pt.header.frame_id = 'base_link'
        pt.header.stamp = self.get_clock().now().to_msg()
        

        # Get 2D position of object in image and transform to pointcloud 3D point
        if self.det_image_pub.get_subscription_count():
            det_result = self.detection_model(array)
            for result in det_result:
                obj_pos_x = result.boxes.xywh[0][0]
                obj_pos_y = result.boxes.xywh[0][1]

                obj_pos_arr.append((obj_pos_x, obj_pos_y))

            det_annotated = det_result[0].plot(show=False)
            
            self.det_image_pub.publish(ros2_numpy.msgify(Image, det_annotated, encoding='rgb8'))

        self.det_image_pub.publish(data)

    def pcl_to_img(data):
        img = cv2.Mat(data.width, data.height, cv2.CV_8UC3) 



def main():
    rclpy.init()

    node = ObjectDetection()
    rclpy.spin(node)

    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
