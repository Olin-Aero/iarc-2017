#!/usr/bin/env python

""" A demo that shows how keypoint matches work using SIFT """

import cv2
import pickle
import numpy as np
import rospkg


class KeyPointMatcherDemo(object):
    """ KeyPointMatcherDemo shows the basics of interest point detection,
        descriptor extraction, and descriptor matching in OpenCV """
    def __init__(self, im1_file, im2_file, descriptor_name):
        rospack = rospkg.RosPack()
        self.im1_file = rospack.get_path('computer_vision_examples') + '/images/' + im1_file
        self.im2_file = rospack.get_path('computer_vision_examples') + '/images/' + im2_file

        # hack to get around OpenCV2 versus OpenCV3 API Changes
        if cv2.__version__ == '3.1.0-dev':
            self.detector = cv2.xfeatures2d.SIFT_create()
            self.extractor = self.detector
        else:
            self.detector = cv2.FeatureDetector_create(descriptor_name)
            self.extractor = cv2.DescriptorExtractor_create(descriptor_name)

        self.matcher = cv2.BFMatcher()
        self.im = None

        self.corner_threshold = 0.0
        self.ratio_threshold = 1.0

    def compute_matches(self):
        """ reads in two image files and computes possible matches between them using SIFT """
        im1 = cv2.imread(self.im1_file)
        im2 = cv2.imread(self.im2_file)

        im1_bw = cv2.cvtColor(im1,cv2.COLOR_BGR2GRAY)
        im2_bw = cv2.cvtColor(im2,cv2.COLOR_BGR2GRAY)

        kp1 = self.detector.detect(im1_bw)
        kp2 = self.detector.detect(im2_bw)

        dc, des1 = self.extractor.compute(im1_bw,kp1)
        dc, des2 = self.extractor.compute(im2_bw,kp2)

        matches = self.matcher.knnMatch(des1,des2,k=2)

        good_matches = []
        for m,n in matches:
            # make sure the distance to the closest match is sufficiently better than the second closest
            if (m.distance < self.ratio_threshold*n.distance and
                kp1[m.queryIdx].response > self.corner_threshold and
                kp2[m.trainIdx].response > self.corner_threshold):
                good_matches.append((m.queryIdx, m.trainIdx))

        pts1 = np.zeros((len(good_matches),2))
        pts2 = np.zeros((len(good_matches),2))

        for idx in range(len(good_matches)):
            match = good_matches[idx]
            pts1[idx,:] = kp1[match[0]].pt
            pts2[idx,:] = kp2[match[1]].pt

        self.im = np.array(np.hstack((im1,im2)))

        # plot the points
        for i in range(pts1.shape[0]):
            cv2.circle(self.im,(int(pts1[i, 0]),int(pts1[i, 1])), 2, (255, 0, 0), 2)
            cv2.circle(self.im,(int(pts2[i, 0] + im1.shape[1]), int(pts2[i, 1])), 2, (255, 0, 0), 2)
            cv2.line(self.im, (int(pts1[i, 0]), int(pts1[i, 1])), (int(pts2[i, 0] + im1.shape[1]), int(pts2[i, 1])), (0, 255, 0))

def set_corner_threshold(thresh):
    """ Sets the threshold to consider an interest point a corner.  The higher the value
        the more the point must look like a corner to be considered """
    global matcher
    matcher.corner_threshold = thresh/1000.0

def set_ratio_threshold(ratio):
    """ Sets the ratio of the nearest to the second nearest neighbor to consider the match a good one """
    global matcher
    matcher.ratio_threshold = ratio/100.0

def mouse_event(event,x,y,flag,im):
    """ Handles mouse events.  In this case when the user clicks, the matches are recomputed """
    if event == cv2.EVENT_FLAG_LBUTTON:
        matcher.compute_matches()

if __name__ == '__main__':
    # descriptor can be: SIFT, SURF, BRIEF, BRISK, ORB, FREAK
    matcher = KeyPointMatcherDemo('frame0000.jpg','frame0001.jpg','SIFT')

    # setup a basic UI
    cv2.namedWindow('UI')
    cv2.createTrackbar('Corner Threshold', 'UI', 0, 100, set_corner_threshold)
    cv2.createTrackbar('Ratio Threshold', 'UI', 100, 100, set_ratio_threshold)
    matcher.compute_matches()

    cv2.imshow("MYWIN",matcher.im)
    cv2.setMouseCallback("MYWIN",mouse_event,matcher)

    while True:
        cv2.imshow("MYWIN",matcher.im)
        cv2.waitKey(50)
    cv2.destroyAllWindows()
