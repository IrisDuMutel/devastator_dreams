#!/usr/bin/env python
# license removed for brevity
import rospy
from mydreams.msg import ObjectDetectionBoxes
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
# import imutils
import message_filters
import os
import sys
import matplotlib.pyplot as plt
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, dir_path)

def SimplePublisher():
    
    rospy.init_node('Realsense_finta', anonymous=True)
    pubrgb = rospy.Publisher("/camera/color/image_raw",Image,queue_size=10)
    pubd = rospy.Publisher("/camera/depth/image_rect_raw", Image, queue_size=1)
    bridge = CvBridge()
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        test_img = cv2.imread("/home/iris/catkin_ws/src/devastator_dreams/images/test.jpg")
        test_img = cv2.resize(test_img, dsize=(0,0), fx=0.1, fy=0.1)
        # print(test_img.shape)
        ### DISPLAY IMAGE CONTENTS ###
        # cv2.namedWindow("img1",cv2.WINDOW_NORMAL)
        # cv2.imshow("img1",test_img)
        # cv2.waitKey(1)
        # cv2.resizeWindow("img1",640,640)

        gray_scale = cv2.imread("/home/iris/catkin_ws/src/devastator_dreams/images/test.jpg",cv2.IMREAD_GRAYSCALE)
        gray_scale = cv2.resize(gray_scale, dsize=(0,0), fx=0.1, fy=0.1)
        # print(gray_scale.shape)
        ### DISPLAY IMAGE CONTENTS ###
        # cv2.namedWindow("img2",cv2.WINDOW_NORMAL)
        # cv2.imshow("img2",gray_scale)
        # cv2.waitKey(1)
        # cv2.resizeWindow("img2",640,640)
        
        test_img = bridge.cv2_to_imgmsg(test_img)
        gray_scale = bridge.cv2_to_imgmsg(gray_scale)
        pubrgb.publish(test_img)
        pubd.publish(gray_scale)

if __name__ == '__main__':
    try:
        SimplePublisher()
    except rospy.ROSInterruptException:
        pass