#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2


class ROS_CV_Test:
    '''Usage Instructions:
       1. Start ros (roscore)
       2. Start webcam (roslaunch usb_cam usb_camera.launch)
       3. Run this script (python ROS_CV_TEST.py)
    '''

    def __init__(self):
        rospy.init_node('ROS_CV_Test')
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_raw_callback)
	self.bridge = CvBridge()

    def image_raw_callback(self, msg):
        try:
            # Read image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Filter pixels
            lower = np.array([150,150,150])
            upper = np.array([255,255,255])
            mask = cv2.inRange(frame, lower, upper)
            frame = cv2.bitwise_and(frame, frame, mask=mask)
            # Display image
            cv2.imshow('frame',frame)
        except CvBridgeError as e:
            print(e)
        cv2.waitKey(3)
      
    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    test = ROS_CV_Test()
    test.run()
