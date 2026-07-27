import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
import numpy as np

from stretch_ar.msg import ImageTarget
from geometry_msgs.msg import Vector3


bridge = CvBridge()

def canny_test():
    cv_image = cv2.imread('questSampleImg.jpg')
    # cv2.circle(cv_image, (int(msg.position.x), int(msg.position.y)), 5, color=(0, 255, 0), thickness=-1)

    upper = 150
    lower = 40

    # get canny image to get outlines
    # cv_canny = self.canny_detector(cv_image, 25, 80)
    cv_image = cv2.resize(cv_image, (1280, 960))
    cv_canny = cv2.addWeighted(cv_image, 1.5, np.zeros(cv_image.shape, cv_image.dtype), 0, 0)
    # cv_canny = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    
    cv_canny = cv2.GaussianBlur(cv_image, (3,3), .6)
    
    # cv_canny = cv2.bilateralFilter(cv_image, 10, 50, 50)
    cv_canny = cv2.Canny(cv_canny, lower, upper)
    # lines = cv2.HoughLinesP(cv_canny, 1, np.pi/180, 68, minLineLength=15, maxLineGap=250)
    
    # cv_canny = canny_detector(cv_image)
    cv_canny = cv2.dilate(cv_canny, (5, 5), iterations=1)
    # for line in lines:
    #     x1, y1, x2, y2 = line[0]
    #     cv2.line(cv_canny, (x1, y1), (x2, y2), (255, 0, 0), 1)
    

    contours, hierarchy = cv2.findContours(cv_canny, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            
    cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 1)

    cv2.imshow('canny', cv_canny)
    cv2.imshow('Contours', cv_image)

    cv2.waitKey(1)

def edge_test():
    cv_image = cv2.imread('questSampleImg.jpg')
    cv_image = cv2.resize(cv_image, (1280, 960))
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # threshold to binary
    thresh = cv2.threshold(cv_image, 0, 255, cv2.THRESH_BINARY)[1]

    # apply morphology
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

    # find contours - write black over all small contours
    letter = morph.copy()
    cntrs = cv2.findContours(morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cntrs = cntrs[0] if len(cntrs) == 2 else cntrs[1]
    for c in cntrs:
        area = cv2.contourArea(c)
        if area < 100:
            cv2.drawContours(letter,[c],0,(0,0,0),-1)

    # do canny edge detection
    edges = cv2.Canny(letter, 200, 200)    

    cv2.imshow('edge', cv_image)
    cv2.waitKey(1)

def canny_detector(img, t_lower=None, t_upper=None): # taken from geeksforgeeks.org https://www.geeksforgeeks.org/machine-learning/implement-canny-edge-detector-in-python-using-opencv/ 
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 

    img = cv2.GaussianBlur(img, (5, 5), 1.4)

    gx = cv2.Sobel(np.float32(img), cv2.CV_64F, 1, 0, 3)
    gy = cv2.Sobel(np.float32(img), cv2.CV_64F, 0, 1, 3)

    mag, ang = cv2.cartToPolar(gx, gy, angleInDegrees=True)

    height, width = img.shape

    mag_max = np.max(mag)
    if t_lower is None:
        t_lower = mag_max * 0.1
    if t_upper is None:
        t_upper = mag_max * 0.5

    nms = np.zeros_like(mag)

    for i_x in range(1, width-1):
        for i_y in range(1, height-1):

            grad_ang = ang[i_y, i_x]
            grad_ang = grad_ang % 180

            if (0 <= grad_ang < 22.5) or (157.5 <= grad_ang <= 180):
                before = mag[i_y, i_x - 1]
                after  = mag[i_y, i_x + 1]

            elif (22.5 <= grad_ang < 67.5):
                before = mag[i_y - 1, i_x + 1]
                after  = mag[i_y + 1, i_x - 1]

            elif (67.5 <= grad_ang < 112.5):
                before = mag[i_y - 1, i_x]
                after  = mag[i_y + 1, i_x]

            else:  # 112.5 - 157.5
                before = mag[i_y - 1, i_x - 1]
                after  = mag[i_y + 1, i_x + 1]

            if mag[i_y, i_x] >= before and mag[i_y, i_x] >= after:
                nms[i_y, i_x] = mag[i_y, i_x]
            else:
                nms[i_y, i_x] = 0

    result = np.zeros_like(nms)

    strong = 255
    weak = 75

    for i_x in range(width):
        for i_y in range(height):
            val = nms[i_y, i_x]

            if val >= t_upper:
                result[i_y, i_x] = strong
            elif val >= t_lower:
                result[i_y, i_x] = weak
            else:
                result[i_y, i_x] = 0

    return result


def main():
    while True:
        canny_test()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
