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
        cv2.namedWindow('video_window')
        cv2.waitKey(5)
        cv2.setMouseCallback('video_window', self.process_mouse_event)
    def find_bounding_boxes(self, image):
        """
        Input: OpenCV image (numpy array)
        Output: List of bounding boxes (topleft, bottomright, isRed)
        """
        # TODO: Make this function
        self.cv_image = cv2.imread(image)
        cv2.namedWindow('threshold_image')
        cv2.createTrackbar('red lower bound', 'threshold_image', 0, 255, self.set_red_lower_bound)
        while(True):
	        binary_image = cv2.inRange(self.cv_image, np.array([self.red_lower_bound,128,128],dtype = "uint8"),np.array([255,255,255],dtype = "uint8"))
	        cv2.namedWindow('threshold_ima')
	        cv2.imshow("images",binary_image)
	        if(cv2.waitKey(1) & 0xFF == ord('q')):
	       		break
		cv2.destroyAllWindows()
        print(binary_image)
        return [((20,20),(30,40),True)]
    def set_red_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.red_lower_bound = val

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        image_info_window = 255*np.ones((500,500,3))
        cv2.putText(image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(5)
def testImageFromFile(filename):
    tracker = ColorTracker()
    # TODO: Load image
    tracker.find_bounding_boxes(filename)
    # TODO: Display result

if __name__ == '__main__':
#	colortracker = ColorTrackerROS()
#	colortracker.run()
    testImageFromFile(sys.argv[1])
