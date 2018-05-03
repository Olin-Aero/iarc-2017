#!/usr/bin/env python2

"""
How to Use:
1. Start ros (roscore)
2. Start webcam (roslaunch usb_cam usb_camera.launch)
3. Run this script (rosrun iarc_vision colortracking)

Owner:


Notes:
If the image is compressed:
rosrun image_transport republish in:=ardrone/bottom/image_raw _image_transport:=compressed raw out:=ardrone/bottom/image_raw

"""

import rospy
import sys
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import Vector3, Vector3Stamped, PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import numpy.linalg as la
import cv2
import math
from image_geometry import PinholeCameraModel
from tf import TransformListener
from tf.transformations import quaternion_from_euler
from iarc_main.msg import Roomba, RoombaList


class ColorTrackerROS(object):
    def __init__(self):
        rospy.init_node('color_tracker')
        self.tracker = ColorTracker()
        self.cv_image = self.processed_image = None  # the latest image from the camera
        self.boxes = []
        self.bridge = CvBridge()  # used to convert ROS messages to OpenCV
        self.cameraModel = PinholeCameraModel()
        self.tf = TransformListener()
        rospy.Subscriber("/ardrone/bottom/image_raw", Image, self.process_image)
        rospy.Subscriber("/ardrone/bottom/camera_info", CameraInfo, self.on_camera_info)
        print("Initializing Color Tracker")
        cv2.namedWindow('preview_window')
        cv2.namedWindow('binary')

        self.debug_pub = rospy.Publisher("tracker/debug", PoseWithCovarianceStamped, queue_size=10)
        self.roomba_pub = rospy.Publisher("visible_roombas", RoombaList, queue_size = 10)

    def process_image(self, msg):
        """
        Process image messages from ROS and stash them in an attribute
        called cv_image for subsequent processing

        """
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # do image processing here
            self.boxes, self.processed_image = self.tracker.find_bounding_boxes(self.cv_image, display=False)
            listOfRoombas = []
            for box in self.boxes:
                center = np.mean(box, axis=0)
                heading = get_heading(box, center, self.processed_image)
                ray = self.cameraModel.projectPixelTo3dRay(center)
                camera_ray = Vector3Stamped(header=msg.header,
                                            vector=Vector3(*ray))
                world_ray = self.tf.transformVector3('map', camera_ray)
                # print(camera_ray, world_ray)
                pos, quat = self.tf.lookupTransform('map', msg.header.frame_id, msg.header.stamp)
                multiplier = -pos[2] / world_ray.vector.z
                drone_to_roomba = np.array([world_ray.vector.x, world_ray.vector.y, world_ray.vector.z]) * multiplier
                quat = quaternion_from_euler(0,0,heading)
                map_to_roomba = pos + drone_to_roomba
                print map_to_roomba

                pose = Pose(position=Point(*map_to_roomba), orientation = Quaternion(*quat))

                pwcs = PoseWithCovarianceStamped(header=Header(frame_id='map', stamp=msg.header.stamp),
                                                 pose=PoseWithCovariance(
                                                     pose=pose,
                                                     covariance=np.diag([.2, .2, 0, 0, 0, 0.2]).flatten()
                                                 ))
                rb = Roomba(last_seen=msg.header.stamp, frame_id='map', type=Roomba.RED, # << TODO:FIX
                        visible_location=pwcs)

                listOfRoombas.append(rb)
                self.debug_pub.publish(pwcs)
            if(not not listOfRoombas):
                self.roomba_pub.publish(header=Header(frame_id='map',stamp=msg.header.stamp),data=listOfRoombas)
            # TODO: Do something with the boxes
            # TODO: make sure it doesn't get too far behind

            # self.binary_image = binary_image
            # self.cv_image = cv_image
        except CvBridgeError as e:
            print "Error loading image"
            print(e)

    def on_camera_info(self, msg):
        self.cameraModel.fromCameraInfo(msg)

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            # start out not issuing any motor commands
            if not self.cv_image is None:
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
        Output: List of bounding boxes (topleft, bottomright, isRed)
        """
        # Get bounding boxes around red and green rectangles
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        red_image = self.get_red_bounding_boxes(hsv_image, image)
        #green_image = self.get_green_bounding_boxes(hsv_image, image)
        # TODO: Differentiate red vs green boxes
        #binary_image = cv2.bitwise_or(red_image, green_image)
        binary_image = red_image
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
        boxes = []
        for i in maxcnt:
            rect = cv2.minAreaRect(i)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            boxes.append(box)
            cv2.drawContours(image, [box], 0, (0, 0, 255), 2)

        if display:
            cv2.imshow("Binary Image With Morphology", binary_image)
            # cv2.imshow("Original images", image)
            # cv2.imshow("HSV image", hsv_image)
            cv2.waitKey(0)
        return boxes, binary_image

    def get_green_bounding_boxes(self, hsv_image, rgb_image):
        binary_image1 = cv2.inRange(hsv_image, np.array([40, 30, 30], dtype="uint8"),
                                    np.array([100, 255, 255], dtype="uint8"))
        return cv2.bitwise_or(binary_image1, binary_image1)

    def get_red_bounding_boxes(self, hsv_image, rgb_image):
        binary_image1 = cv2.inRange(hsv_image, np.array([0, 100, 0], dtype="uint8"),
                                    np.array([5, 255, 255], dtype="uint8"))
        binary_image2 = cv2.inRange(hsv_image, np.array([170, 100, 0], dtype="uint8"),
                                    np.array([180, 255, 255], dtype="uint8"))
        # binary_image3 = cv2.inRange(rgb_image, np.array([80, 80, 250], dtype="uint8"),
        #                             np.array([220, 230, 255], dtype="uint8"))
        binary_image4 = cv2.bitwise_or(binary_image1, binary_image2)
        # return cv2.bitwise_or(binary_image3, binary_image4)
        return binary_image4

def get_heading(box, center, binary_image):
    """
    Ox is 0 degrees, rotate clockwise from Ox results in positive angles
    rotate counter-clockwise results in negative angles
    :param box: bounding box of the roomba
    :param center: center of the boundbing box
    :param binary_image: image containing roombas
    :return: orientation in radian
    """
    # TODO: Check extreme case when 3 corners are black (due to glaring effect)
    # TODO: bounding box returns negative values and causes crash when trying to get binary_image[i][j]

    # Bounding box always returns rectangle points in clockwise direction
    darkness_corners = get_darkness_corners(box, center, binary_image)

    # Get shorter side's orientation
    if distance(box[0], box[1]) < distance(box[1], box[2]):
        if darkness_corners[0] is True:
            # heading from box[1] -> box[0]
            heading = get_vector_heading(box[1], box[0])
            print "box 1 to 0"
        else:
            # heading from box[0] -> box[1]
            heading = get_vector_heading(box[0], box[1])
            print "box 0 to 1"
    else:
        if darkness_corners[1] is True:
            # heading from box[2] -> box[1]
            heading = get_vector_heading(box[2], box[1])
            print "box 2 to 1"
        else:
            # heading from box[1] -> box[2]
            heading = get_vector_heading(box[1], box[2])
            print "box 1 to 2"
    heading = (-heading) - math.pi / 2
    if(heading < math.pi):
        heading = heading + 2*math.pi
    print "Heading angle", math.degrees(heading), heading
    return heading


def get_vector_heading(tail, head):
    """
    Find orientation of a vector defined by tail and head
    :param tail:
    :param head:
    :return: orientation of the vector.
    """
    # The input vector
    u = head - tail
    return np.arctan2(u[1], u[0])


def get_darkness_corners(box, center, binary_image):
    """
    Evaluate 4 corners and check which two of these are potentially black
    Does not check extreme case where more than two corners are black (glaring effect)
    :param box: bounding box of the roomba
    :param center: center of the boundbing box
    :param binary_image: image containing roombas
    :return: a list indicating whether a corner is black or white.
    """
    corner1_darkness = get_average_darkness(box[0], center, binary_image)
    corner2_darkness = get_average_darkness(box[1], center, binary_image)
    corner3_darkness = get_average_darkness(box[2], center, binary_image)
    corner4_darkness = get_average_darkness(box[3], center, binary_image)
    list = [corner1_darkness, corner2_darkness, corner3_darkness, corner4_darkness]
    list = sorted(list)
    return [corner1_darkness <= list[1], corner2_darkness <= list[1], corner3_darkness <= list[1],
            corner4_darkness <= list[1]]


def get_average_darkness(point_a, point_b, binary_image):
    """
    Return the average pixel of the diagonal connecting A and B
    :param point_a: first point define the tail of vector AB
    :param point_b: second point define the head of vector AB
    :param binary_image: image of 0s and 1s
    :return: from 0 (dark) to 1 (light)
    :rtype: float
    """
    mid = (point_a + point_b) / 2
    quarter = (point_a + mid) / 2
    if point_a[0] > quarter[0]:
        y0 = quarter[0]
        y1 = point_a[0]
    else:
        y0 = point_a[0]
        y1 = quarter[0]
    if point_a[1] > quarter[1]:
        x0 = quarter[1]
        x1 = point_a[1]
    else:
        x0 = point_a[1]
        x1 = quarter[1]

    intensity = 0
    count = 0
    for i in range(int(x0), int(x1)):
        for j in range(int(y0), int(y1)):
            # TODO: 100 here is an arbitrary choice, need a better way to get pixels lying on the diagonal
            if abs((i - x0) * (point_a[0] - quarter[0]) - (j - y0) * (point_a[1] - quarter[1])) <= 100:
                if i < len(binary_image) and j < len(binary_image[i]) and binary_image[i][j] > 0:
                    intensity += 1
                count += 1
    if count == 0:
        return 0

    return float(intensity) / count


def test_image_from_file(filename):
    tracker = ColorTracker()
    # TODO: Load image
    boxes, binary_image = tracker.find_bounding_boxes(filename, True)

    # TODO: Display result


def distance(p0, p1):
    """
    Find distance between two 2D points
    :param p0: point 1
    :param p1: point 2
    :return: distance
    :rtype: float
    """
    return np.sqrt((p0[0] - p1[0]) ** 2 + (p0[1] - p1[1]) ** 2)


if __name__ == '__main__':
    colortracker = ColorTrackerROS()
    colortracker.run()
    # test_image_from_file(cv2.imread(sys.argv[1]))
