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
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, dir_path)

def SimplePublisher():
    rospy.init_node('Realsense_finta', anonymous=True)
    pubrgb = rospy.Publisher("/camera/color/image_raw",Image,queue_size=10)
    # pubd = rospy.Publisher("/camera/color/image_rect_raw", Image, queue_size=1, buff_size=2**24)
    bridge = CvBridge()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        test_img = cv2.imread("/home/iris/catkin_ws/src/devastator_dreams/images/test.jpg")
        test_img = bridge.imgmsg_to_cv2(test_img, desired_encoding="passthrough")
        grey_scale = cv2.imread("../images/test.jpg",IMREAD_GRAYSCALE)
        pubrgb.publish(test_img)
        pubd.publish()

if __name__ == '__main__':
    try:
        SimplePublisher()
    except rospy.ROSInterruptException:
        pass