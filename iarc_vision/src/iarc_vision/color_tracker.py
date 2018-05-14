#!/usr/bin/env python2
import sys
import numpy as np
import numpy.linalg as la
import cv2

class ColorTracker(object):
    def __init__(self,
            red_bound_1 = ( (0,100,0), (5,255,255) ),
            red_bound_2 = ( (170,100,0), (180,255,255) ),
            green_bound = ( (30,0,0), (100,255,255) ),
            aspect_bound = (1.5, 2.0)
            ):
        """
        Initialize Color tracker with threshold parameters.
        red_bound : (low,high) hsv bounds for red plate
        green_bound : (low,high) hsv bounds for green plate
        aspect_bound : (low,high) aspect-ratio bound for plates
        """
        self._red_bound_1 = red_bound_1
        self._red_bound_2 = red_bound_2
        self._green_bound = green_bound
        self._aspect_bound = aspect_bound

    def find_bounding_boxes(self, image, min_area):
        """
        Input: OpenCV image (numpy array)
        Output: List of bounding boxes (topleft, bottomright, isRed)
        """
        # Get bounding boxes around red and green rectangles
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        red_image = self.get_red_bounding_boxes(hsv_image, image)
        green_image = self.get_green_bounding_boxes(hsv_image, image)

        # TODO: Differentiate red vs green boxes
        bin_img = cv2.bitwise_or(red_image, green_image)
        #bin_img = red_image
        # Remove noise
        kernel = np.ones((5, 5), np.uint8)
        bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, kernel)

        # Threshold and find contours
        ret, thresh = cv2.threshold(bin_img, 127, 255, 0)
        _, contours, hierarchy = cv2.findContours(thresh,
                cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Get bounding box from contours
        maxcnt = []
        for i in contours:
            a = cv2.contourArea(i)
            if a > min_area:
                maxcnt.append(i)

        # Draw coutours on the image
        boxes = []
        for i in maxcnt:
            rect = cv2.minAreaRect(i)
            width, height = rect[1]

            box = cv2.boxPoints(rect)
            box = np.int0(box)

            a1 = float(height) / width
            a2 = float(width) / height
            aspect = max(a1,a2)

            if self._aspect_bound[0] < aspect < self._aspect_bound[1]:
                boxes.append(box)
                cv2.drawContours(image, [box], 0, (0,0,255), 2)

        return boxes, bin_img

    def get_green_bounding_boxes(self, hsv_image, rgb_image):
        return cv2.inRange(hsv_image, self._green_bound[0], self._green_bound[1])

    def get_red_bounding_boxes(self, hsv_image, rgb_image):
        b1 = cv2.inRange(hsv_image, self._red_bound_1[0], self._red_bound_1[1])
        b2 = cv2.inRange(hsv_image, self._red_bound_2[0], self._red_bound_2[1])
        return cv2.bitwise_or(b1,b2)

if __name__ == "__main__":
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('file', type=str)
    args = parser.parse_args()
    img = cv2.imread(args.file)

    tracker = ColorTracker()
    boxes, bin_img = tracker.find_bounding_boxes(img, 1000)
    print boxes
    cv2.imshow('binary', bin_img)
    cv2.imshow('proc', img)
    cv2.waitKey(0)
