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
        print("Initializing Color Tracker")
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
        self.red_lower_bound = 0
    def find_bounding_boxes(self, image):
        """
        Input: OpenCV image (numpy array)
        Output: List of bounding boxes (topleft, b;;ottomright, isRed)
        """
        # TODO: Make this function
        self.cv_image = cv2.imread(image)
        hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        binary_image1 = cv2.inRange(hsv_image, np.array([0,100,0],dtype = "uint8"),np.array([10,255,255],dtype = "uint8"))
        binary_image2 = cv2.inRange(hsv_image, np.array([170,100,0],dtype = "uint8"),np.array([180,255,255],dtype = "uint8"))
        binary_image3 = cv2.inRange(self.cv_image, np.array([80,80,250],dtype = "uint8"),np.array([220,230,255],dtype = "uint8"))
        binary_image4 = cv2.bitwise_or(binary_image1,binary_image2)
        binary_image5 = cv2.bitwise_or(binary_image3,binary_image4)
        kernel = np.ones((5,5),np.uint8)
        binary_image6 = cv2.morphologyEx(binary_image5, cv2.MORPH_OPEN, kernel)
        #cv2.imshow("images",binary_image)
        #cv2.waitKey(0)
        ret,thresh = cv2.threshold(binary_image3,127,255,0)
        image,contours,hierarchy = cv2.findContours(thresh,1,2)
        #cnt = countours[0]
        maxcnt = []
        for i in contours:
        	if(cv2.contourArea(i) > 1000):
        		maxcnt.append(i)
        for i in maxcnt:
	        rect = cv2.minAreaRect(i)
	    	box = cv2.boxPoints(rect)
	    	box = np.int0(box)
	    	cv2.drawContours(self.cv_image,[box],0,(0,0,255),2)
    	cv2.imshow("HSV image",hsv_image)
        #cv2.waitKey(0)

        cv2.imshow("Binary Image BRG",binary_image3)
        cv2.imshow("Binary Image HSV",binary_image4)
        cv2.imshow("Binary Image With Morphology",binary_image6)
        cv2.imshow("images",self.cv_image)
        cv2.waitKey(0)
        return [((20,20),(30,40),True)]
def set_red_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.red_lower_bound = val
def testImageFromFile(filename):
    tracker = ColorTracker()
    # TODO: Load image
    tracker.find_bounding_boxes(filename)
    # TODO: Display result

if __name__ == '__main__':
#	colortracker = ColorTrackerROS()
#	colortracker.run()
    testImageFromFile(sys.argv[1])
