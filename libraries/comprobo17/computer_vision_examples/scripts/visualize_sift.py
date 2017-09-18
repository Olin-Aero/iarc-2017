#!/usr/bin/env python


""" A simple script to visualize the SIFT descriptor for an interactively drawn sketch """

import numpy as np
import cv2
from math import pi, cos, sin
import rospy

drawing = False
V = None
visualizer_size = (512,512)
patch_size = (128,128)
# we will draw our patch on im
im =  255*np.ones(patch_size,dtype=np.uint8)

def visualize_descriptor():
    """ Visualize the current value of the SIFT descriptor """
    global V
    dc, des = extractor.compute(im, [keypoint])

    # hardcoded, there are 16 orientation histograms with 8 orientations each
    O = des[0].reshape((16,8))
    V = 255*np.ones(visualizer_size,dtype=np.uint8)

    # draw the grid that divides the different histograms
    for i in range(0,V.shape[0]+1,V.shape[0]/4):
        cv2.line(V,(0,i),(V.shape[1],i),(0,0,0),2)
        cv2.line(V,(i,0),(i,V.shape[0]),(0,0,0),2)

    # loop over all columns of the grid
    for i in range(4):
        # loop over all rows of the grid
        for j in range(4):
            # loop over all orientations
            for k in range(8):
                theta = k*pi/4.0
                center = ((i+0.5)*V.shape[0]/4.0,(j+0.5)*V.shape[0]/4.0)
                v = O[i+j*4,k]/10.0
                end_point = ((i+0.5)*V.shape[0]/4.0+v*cos(theta),(j+0.5)*V.shape[0]/4.0-v*sin(theta))

                # compute the end points of the lines on the arrow head
                arrow_e1 = (end_point[0]+cos(theta+3*pi/4)*v/4.0, end_point[1]-sin(theta+3*pi/4)*v/4.0)
                arrow_e2 = (end_point[0]+cos(theta-3*pi/4)*v/4.0, end_point[1]-sin(theta-3*pi/4)*v/4.0)

                # draw a line with an arrow head
                cv2.line(V,(int(center[0]),int(center[1])),(int(end_point[0]),int(end_point[1])),2)
                cv2.line(V,(int(end_point[0]), int(end_point[1])), (int(arrow_e1[0]),int(arrow_e1[1])),2)
                cv2.line(V,(int(end_point[0]), int(end_point[1])),(int(arrow_e2[0]),int(arrow_e2[1])),2)

def mouse_event(event,x,y,flag,dc):
    """ handle mouse events, basically lets you sketch by clicking in the left pane """
    global drawing
    global last_x
    global last_y
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        last_x = x
        last_y = y
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
    elif event == cv2.EVENT_MOUSEMOVE and drawing:
        # draw a line between the last mouse position and the current one
        cv2.line(im,(int(x/(visualizer_size[1]/patch_size[1])),int(y/(visualizer_size[0]/patch_size[0]))),(int(last_x/(visualizer_size[1]/patch_size[1])),int(last_y/(visualizer_size[0]/patch_size[0]))),0,2)
        visualize_descriptor()
        last_x = x
        last_y = y

if __name__ == '__main__':
    # hack to get around OpenCV2 versus OpenCV3 API Changes
    if cv2.__version__ == '3.1.0-dev':
        detector = cv2.xfeatures2d.SIFT_create()
        extractor = detector
    else:
        detector = cv2.FeatureDetector_create('SIFT')
        extractor = cv2.DescriptorExtractor_create('SIFT')

    keypoint = cv2.KeyPoint()
    keypoint.pt = ((im.shape[1]-1)/2.0, (im.shape[0]-1)/2.0)
    keypoint.octave = 1245440   # this is weird because multiple things are encoded here using bitmasking
    keypoint.size = 40.0        # this controls the precision of the Gaussian window
    keypoint.angle = 0.0        # rotation invariance is given by the keypoint not the descriptor

    cv2.namedWindow("mywin")
    cv2.setMouseCallback("mywin",mouse_event)
    visualize_descriptor()

    print "Draw on the canvas by clicking and holding the mouse (move slowly)"
    print "Reset the sketch by pressing the spacebar"

    while not rospy.is_shutdown():
        cv2.imshow("mywin", np.hstack((cv2.resize(im,visualizer_size),V)))
        key = cv2.waitKey(25)
        if key != -1 and chr(key) == ' ':
            # if you hit space bar, you should resest the sketch on the left
            im =  255*np.ones(patch_size,dtype=np.uint8)
            visualize_descriptor()