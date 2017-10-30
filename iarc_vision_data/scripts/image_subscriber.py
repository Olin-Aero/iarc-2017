import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import numpy as np

class ImageSubscriber(object):
    def __init__(self):
        super(ImageSubscriber, self).__init__()
        self.cv_image = None                        # the latest image from the camera

        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        cv2.namedWindow('video_window')
        rospy.Subscriber("/camera/image_raw", Image, self.process_image)
        print "Initialize ImageSubscriber"

    def lower_half_image(self, array):
        """
        Returns the lower half rows of an image
        Args: array (array): the array you want to extract the lower half from
        Returns: The lower half of the array
        """
        return array[round(array.shape[0]/2):,:,:]

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
        called cv_image for subsequent processing """
        # print "Image callback"
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image = self.lower_half_image(cv_image)
        cv_image = cv2.resize(cv_image, (64, 64))
        self.cv_image = cv_image
