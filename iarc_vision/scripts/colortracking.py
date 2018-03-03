#!/usr/bin/env python2

"""
How to Use:
1. Start ros (roscore)
2. Start webcam (roslaunch usb_cam usb_camera.launch)
3. Run this script (rosrun iarc_vision colortracking)

Owner: 

"""


import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

class ColorTrackerROS(object):
    def __init__(self):
        rospy.init_node('color_tracker')
        self.cv_image = None     # the latest image from the camera
        self.bridge = CvBridge() # used to convert ROS messages to OpenCV
        rospy.Subscriber("/usb_cam/image_raw", Image, self.process_image)
        print "Initializing Color Tracker"
        cv2.namedWindow('preview_window')
        cv2.namedWindow('binary')

        self.tracker = ColorTracker()

    def process_image(self, msg):
        """
        Process image messages from ROS and stash them in an attribute
        called cv_image for subsequent processing
		
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # do image processing here
            boxes = self.tracker.find_bounding_boxes(cv_image)
            
            # TODO: Do something with the boxes
            # TODO: make sure it doesn't get too far behind

            binary_image = cv2.inRange(cv_image, (128,128,128),(255,255,255))
            self.binary_image = binary_image
            self.cv_image = cv_image
        except CvBridgeError as e:
            print "Error loading image"
            print(e)

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            # start out not issuing any motor commands
            if not self.cv_image is None:
                print self.cv_image.shape
                cv2.imshow('preview_window', self.cv_image)
                cv2.imshow('binary', self.binary_image)
                cv2.waitKey(5)
            r.sleep()

class ColorTracker(object):
    def __init__(self):
        pass

    def find_bounding_boxes(self, image):
        """
        Input: OpenCV image (numpy array)
        Output: List of bounding boxes (topleft, bottomright, isRed)
        """
        # TODO: Make this function
        return [((20,20),(30,40),True)]

def testImageFromFile(filename):
    tracker = ColorTracker()
    # TODO: Load image
    tracker.find_bounding_boxes("TODO: This")
    # TODO: Display result

if __name__ == '__main__':
#	colortracker = ColorTrackerROS()
#	colortracker.run()
    testImageFromFile(sys.argv[1])
