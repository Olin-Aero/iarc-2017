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

        self.tracker = ColorTracker()

        self.cv_image = self.processed_image = None  # the latest image from the camera
        self.bridge = CvBridge()  # used to convert ROS messages to OpenCV
        rospy.Subscriber("/ardrone/bottom/image_raw", Image, self.process_image)
        print("Initializing Color Tracker")
        cv2.namedWindow('preview_window')
        cv2.namedWindow('binary')

    def process_image(self, msg):
        """
        Process image messages from ROS and stash them in an attribute
        called cv_image for subsequent processing
        
        """
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # do image processing here
            self.boxes, self.processed_image = self.tracker.find_bounding_boxes(self.cv_image, display=False)

            # TODO: Do something with the boxes
            # TODO: make sure it doesn't get too far behind

            # self.binary_image = binary_image
            # self.cv_image = cv_image
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
                cv2.imshow('binary', self.processed_image)
                cv2.waitKey(5)
            r.sleep()


class ColorTracker(object):
    def __init__(self):
        self.red_lower_bound = 0

    def find_bounding_boxes(self, image, display=True):
        """
        Input: OpenCV image (numpy array)
        Output: List of bounding boxes (topleft, b;;ottomright, isRed)
        """
        # TODO: Make this function
        # Get bounding boxes around red and green rectangles
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        red_image = self.get_red_bounding_boxes(hsv_image, image)
        green_image = self.get_green_bounding_boxes(hsv_image, image)
        binary_image = cv2.bitwise_or(green_image, green_image)

        # Remove noise
        kernel = np.ones((5, 5), np.uint8)
        binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)

        # Threshold and find contours
        ret, thresh = cv2.threshold(binary_image, 127, 255, 0)
        binary_image, contours, hierarchy = cv2.findContours(thresh, 1, 2)

        # Get bounding box from contours
        maxcnt = []
        for i in contours:
            if cv2.contourArea(i) > 1000:
                maxcnt.append(i)

        # Draw coutours on the image
        for i in maxcnt:
            rect = cv2.minAreaRect(i)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(image, [box], 0, (0, 0, 255), 2)

        print contours

        if display:
            cv2.imshow("Original images", image)
            cv2.imshow("HSV image", hsv_image)
            cv2.imshow("Binary Image With Morphology", binary_image)
            cv2.waitKey(0)
        return maxcnt, binary_image

    def get_green_bounding_boxes(self, hsv_image, rgb_image):
        binary_image1 = cv2.inRange(hsv_image, np.array([40, 30, 30], dtype="uint8"),
                                    np.array([100, 255, 255], dtype="uint8"))
        return cv2.bitwise_or(binary_image1, binary_image1)

    def get_red_bounding_boxes(self, hsv_image, rgb_image):
        binary_image1 = cv2.inRange(hsv_image, np.array([0, 100, 0], dtype="uint8"),
                                    np.array([10, 255, 255], dtype="uint8"))
        binary_image2 = cv2.inRange(hsv_image, np.array([170, 100, 0], dtype="uint8"),
                                    np.array([180, 255, 255], dtype="uint8"))
        binary_image3 = cv2.inRange(rgb_image, np.array([80, 80, 250], dtype="uint8"),
                                    np.array([220, 230, 255], dtype="uint8"))
        binary_image4 = cv2.bitwise_or(binary_image1, binary_image2)
        return cv2.bitwise_or(binary_image3, binary_image4)


def set_red_lower_bound(self, val):
    """ A callback function to handle the OpenCV slider to select the red lower bound """
    self.red_lower_bound = val


def testImageFromFile(filename):
    tracker = ColorTracker()
    # TODO: Load image
    tracker.find_bounding_boxes(filename, True)
    # TODO: Display result


if __name__ == '__main__':
    colortracker = ColorTrackerROS()
    colortracker.run()
    # testImageFromFile(cv2.imread(sys.argv[1]))
