#!usr/bin/ python


## This node is in charge of three main tasks:
#    · Receiving images from the RealSense
#    · Stitching such images into a unique panoramic image
#    · Publishing panoramic image into topic to Unity
#    · Moving the head of the Devastator to capture different images

import rospy
from mydreams.msg import ObjectDetectionBoxes
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import imutils
import message_filters
import os
import sys
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, dir_path)
from imgstitcher import Stitcher



class thenode:

    def __init__(self):
        self.movinghead = False          
        # Image directory
        # self.directory = "../images"
        # # Change the current directory 
        # # to specified directory 
        # os.chdir(self.directory)
        # self.bridge = CvBridge()
        # stitch the images together to create a panorama
        self.stitcher = Stitcher()

        rospy.init_node('thenode', anonymous=True)

        self.pubrgb = rospy.Publisher('/panoramicrgb_img',Image,queue_size=10)
        self.pubd = rospy.Publisher('/panoramicd_img',Image,queue_size=10)
    
        self.images_sub   = message_filters.Subscriber("/camera/color/image_raw", Image, queue_size=1, buff_size=2**24)
        self.depth_sub   = message_filters.Subscriber("/camera/depth/image_rect_raw", Image, queue_size=1, buff_size=2**24)
        # self.headflag_sub   = message_filters.Subscriber("/moveHead_flag", bool), self.headflag_sub
        ts = message_filters.ApproximateTimeSynchronizer([self.images_sub, self.depth_sub], queue_size=10, slop=0.5, allow_headerless=True)
        ts.registerCallback(callback,self.pub)

        test_img = cv2.imread("../images/test.jpg")
        test_img = bridge.imgmsg_to_cv2(test_img, desired_encoding="passthrough")
        self.pubrgb.publish(test_img)
    
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

#TODO: check if it is necessary to write self.images_sub,self.headflag_sub
    def callback(self,pubrgb,pubd):
    
        if self.headflag_sub:
            # Start moving head. We only enter this once
            self.movinghead = True
        
        if self.movinghead:
            # Move head a certain amount

    
    
            # Gather images and stitch them
            imagergb_saved = cv2.imread('panoramic_rgb.jpg')
            imaged_saved = cv2.imread('panoramic_depth.jpg')
            imagergb_RS = bridge.imgmsg_to_cv2(self.images_sub, desired_encoding="bgr8")
            imaged_RS = bridge.imgmsg_to_cv2(self.depth_sub, desired_encoding="mono8")

            if imagergb_saved == None:
                # There are no images saved to be compared: save first image
                cv2.imwrite('panoramic_rgb.jpg',imagergb_RS)
                cv2.imwrite('panoramic_depth.jpg',imaged_RS)
            
            else:

                (result_rgb, vis_rgb) = stitcher.stitch([imagergb_RS, imagergb_saved], showMatches=True)
                (result_d, vis_d) = stitcher.stitch([imaged_RS, imaged_saved], showMatches=True)

                # Saving the image
                cv2.imwrite('panoramic_rgb.jpg',result_rgb)
                cv2.imwrite('panoramic_depth.jpg',result_d)

            if motor_limit:
                # Stop head from moving
                self.movinghead=False
                # Publish images
                imgrgb_pub = self.bridge.cv2_to_imgmsg(result_rgb, encoding='passthrough')
                imgd_pub = self.bridge.cv2_to_imgmsg(result_d, encoding='passthrough')
                self.pubrgb.publish(imgrgb_pub)
                self.pubd.publish(imgd_pub)    


if __name__ == '__main__':
    thisnode= thenode()
