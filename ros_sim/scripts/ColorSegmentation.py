#!/usr/bin/env python

# ROS passes around images in its own sensor_msgs/Image message format, but many users will want 
# to use images in conjunction with OpenCV. CvBridge is a ROS library that provides an interface between ROS and OpenCV.

# http://wiki.ros.org/cv_bridge/Tutorials

# https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html


import rospy
import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Camera:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_received = False
        self.image = [[]]

        # Connect image topic
        rospy.Subscriber("/camera/image_raw", Image, self.ImageCallback)
        rospy.sleep(1)

    def ImageCallback(self, data):
        try:
            # To interface ROS and OpenCV by converting ROS images into OpenCV images
            # Convert a sensor_msgs::Image message to an OpenCV cv::Mat.
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_received = True
            self.image = cv_image
        except CvBridgeError as e:
            rospy.logerr(e)        

    def take_photo(self, img_title):
        if self.image_received:
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False

if __name__ == "__main__":
    rospy.init_node("ColorSegmentation", anonymous=False)
    camera = Camera()

    # lower_color = np.array([0, 0, 0])
    # upper_color = np.array([0, 0, 0])

    while not rospy.is_shutdown():
        hsv = cv2.cvtColor(camera.image, cv2.COLOR_BGR2HSV)

                             # H, S, V
        lower_red = np.array([0,120,70])
        upper_red = np.array([10,255,255])
                            
        # Detect the object based on HSV Range Values (find the max of pixels)
                            #src, lower, upper
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170,120,70])
        upper_red = np.array([180,255,255])
        mask2 = cv2.inRange(hsv,lower_red,upper_red)

        mask = mask1 + mask2

        # Take only red colors
                                #src1, src2, mask (changes the output)
                                # out = src1 & src2
        result = cv2.bitwise_and(hsv, hsv, mask = mask)
        result = cv2.cvtColor(result, cv2.COLOR_HSV2BGR)

        cv2.imshow('video', camera.image)
        cv2.imshow('filtered_video', result)
        cv2.waitKey(10) # Display 10ms and close frame