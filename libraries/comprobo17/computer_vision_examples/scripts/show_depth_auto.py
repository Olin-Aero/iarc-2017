#!/usr/bin/env python

import cv2
import pickle
import numpy as np
from math import exp, acos
import rospy
from copy import deepcopy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge


ratio_threshold = 1.0
corner_threshold = 0.0
epipolar_threshold = 0.006737946999085467

W = np.array([[0.0, -1.0, 0.0],
              [1.0, 0.0, 0.0],
              [0.0, 0.0, 1.0]])

colors = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,0,255),(0,255,255)]
pt_num = 0
im1_pts = []
im2_pts = []

def triangulate_points(pt_set1, pt_set2, P, P1):
    my_points = cv2.triangulatePoints(P,P1,pt_set1.T,pt_set2.T)
    projected_points_1 = P.dot(my_points)
    
    # convert to inhomogeneous coordinates
    for i in range(projected_points_1.shape[1]):
        projected_points_1[0,i] /= projected_points_1[2,i]
        projected_points_1[1,i] /= projected_points_1[2,i]
        projected_points_1[2,i] /= projected_points_1[2,i]

    projected_points_2 = P1.dot(my_points)
    # convert to inhomogeneous coordinates
    for i in range(projected_points_2.shape[1]):
        projected_points_2[0,i] /= projected_points_2[2,i]
        projected_points_2[1,i] /= projected_points_2[2,i]
        projected_points_2[2,i] /= projected_points_2[2,i]

    # convert to inhomogeneous coordinates
    for i in range(projected_points_2.shape[1]):
        my_points[0,i] /= my_points[3,i]
        my_points[1,i] /= my_points[3,i]
        my_points[2,i] /= my_points[3,i]
        my_points[3,i] /= my_points[3,i]

    return my_points.T

def test_epipolar(E,pt1,pt2):
    pt1_h = np.zeros((3,1))
    pt2_h = np.zeros((3,1))
    pt1_h[0:2,0] = pt1.T
    pt2_h[0:2,0] = pt2.T
    pt1_h[2] = 1.0
    pt2_h[2] = 1.0
    return pt2_h.T.dot(E).dot(pt1_h)

def test_triangulation(P,pcloud):
    P4x4 = np.eye(4)
    P4x4[0:3,:] = P
    pcloud_3d = pcloud[:,0:3]
    projected = cv2.perspectiveTransform(np.array([pcloud_3d]),P4x4)
    return np.mean(projected[0,:,2]>0.0)

def compute_depths():
    temporal_step = 30
    if K == None or D == None or len(images) < temporal_step:
        return
    im1_bw = images[-temporal_step]
    im2_bw = images[-1]
    delta_t = timestamps[-1] - timestamps[-temporal_step]
    global F
    global im
    kp1 = detector.detect(im1_bw)
    kp2 = detector.detect(im2_bw)

    dc, des1 = extractor.compute(im1_bw,kp1)
    dc, des2 = extractor.compute(im2_bw,kp2)

    # do matches both ways so we can better screen out spurious matches
    matches = matcher.knnMatch(des1,des2,k=2)
    matches_reversed = matcher.knnMatch(des2,des1,k=2)

    # apply the ratio test in one direction
    good_matches_prelim = []
    for m,n in matches:
        if m.distance < ratio_threshold*n.distance and kp1[m.queryIdx].response > corner_threshold and kp2[m.trainIdx].response > corner_threshold:
            good_matches_prelim.append((m.queryIdx, m.trainIdx))

    # apply the ratio test in the other direction
    good_matches = []
    for m,n in matches_reversed:
        if m.distance < ratio_threshold*n.distance and (m.trainIdx,m.queryIdx) in good_matches_prelim:
            good_matches.append((m.trainIdx, m.queryIdx))

    auto_pts1 = np.zeros((1,len(good_matches),2))
    auto_pts2 = np.zeros((1,len(good_matches),2))

    for idx in range(len(good_matches)):
        match = good_matches[idx]
        auto_pts1[0,idx,:] = kp1[match[0]].pt
        auto_pts2[0,idx,:] = kp2[match[1]].pt

    auto_pts1_orig = auto_pts1
    auto_pts2_orig = auto_pts2

    # remove the effect of the intrinsic parameters as well as radial distortion
    auto_pts1 = cv2.undistortPoints(auto_pts1, K, D)
    auto_pts2 = cv2.undistortPoints(auto_pts2, K, D)

    correspondences = [[],[]]
    for i in range(auto_pts1_orig.shape[1]):
        correspondences[0].append((auto_pts1_orig[0,i,0],auto_pts1_orig[0,i,1]))
        correspondences[1].append((auto_pts2_orig[0,i,0],auto_pts2_orig[0,i,1]))

    im1_pts = np.zeros((len(correspondences[0]),2))
    im2_pts = np.zeros((len(correspondences[1]),2))

    im = np.array(np.hstack((im1_bw,im2_bw)))

    # plot the points
    for i in range(len(im1_pts)):
        im1_pts[i,0] = correspondences[0][i][0]
        im1_pts[i,1] = correspondences[0][i][1]
        im2_pts[i,0] = correspondences[1][i][0]
        im2_pts[i,1] = correspondences[1][i][1]

        cv2.circle(im,(int(im1_pts[i,0]),int(im1_pts[i,1])),2,(255,0,0),2)
        cv2.circle(im,(int(im2_pts[i,0]+im1_bw.shape[1]),int(im2_pts[i,1])),2,(255,0,0),2)

    # the np.array bit makes the points into a 1xn_pointsx2 numpy array since that is what undistortPoints requires
    im1_pts_ud = cv2.undistortPoints(np.array([im1_pts]),K,D)
    im2_pts_ud = cv2.undistortPoints(np.array([im2_pts]),K,D)

    # since we are using undistorted points we are really computing the essential matrix
    E, mask = cv2.findFundamentalMat(im1_pts_ud,im2_pts_ud,cv2.FM_RANSAC,epipolar_threshold)

    # correct matches using the optimal triangulation method of Hartley and Zisserman
    im1_pts_ud_fixed, im2_pts_ud_fixed = cv2.correctMatches(E, im1_pts_ud, im2_pts_ud)

    epipolar_error = np.zeros((im1_pts_ud_fixed.shape[1],))
    for i in range(im1_pts_ud_fixed.shape[1]):
        epipolar_error[i] = test_epipolar(E,im1_pts_ud_fixed[0,i,:],im2_pts_ud_fixed[0,i,:])

    # since we used undistorted points to compute F we really computed E, now we use E to get F
    F = np.linalg.inv(K.T).dot(E).dot(np.linalg.inv(K))
    U, Sigma, V = np.linalg.svd(E)

    # these are the two possible rotations
    R1 = U.dot(W).dot(V)
    R2 = U.dot(W.T).dot(V)

    # flip sign of E if necessary
    if np.linalg.det(R1)+1.0 < 10**-8:
        # flip sign of E and recompute everything
        E = -E
        F = np.linalg.inv(K.T).dot(E).dot(np.linalg.inv(K))
        U, Sigma, V = np.linalg.svd(E)

        R1 = U.dot(W).dot(V)
        R2 = U.dot(W.T).dot(V)

    # these are the two possible translations between the two cameras (up to a scale)
    t1 = U[:,2]
    t2 = -U[:,2]

    # the first camera has a camera matrix with no translation or rotation
    P = np.array([[1.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.0]]);
    P1_possibilities = [np.column_stack((R1, t1)),
                        np.column_stack((R1, t2)),
                        np.column_stack((R2, t1)),
                        np.column_stack((R2, t2))]

    pclouds = []
    for P1 in P1_possibilities:
        pclouds.append(triangulate_points(im1_pts_ud_fixed, im2_pts_ud_fixed, P, P1))

    # compute the proportion of points in front of the cameras
    infront_of_camera = []
    for i in range(len(P1_possibilities)):
        infront_of_camera.append(test_triangulation(P,pclouds[i])+test_triangulation(P1_possibilities[i],pclouds[i]))

    # the highest proportion of points in front of the cameras is the one we select
    best_pcloud_idx = np.argmax(infront_of_camera)
    best_pcloud = pclouds[best_pcloud_idx]

    print P1_possibilities[best_pcloud_idx][:,-1]
    print acos((np.trace(P1_possibilities[best_pcloud_idx][:,0:3])-1)/2) / delta_t.to_sec()

    # scale the depths between 0 and 1 so it is easier to visualize
    depths = best_pcloud[:,2] - min(best_pcloud[:,2])
    depths = depths / max(depths)
    cv2.imshow("MYWIN", im)
    image_to_display = deepcopy(rect_images[-1])
    points = [[0,0],[640,0],[640,480],[0,480]]
    for x,y in points:
        l = F.dot(np.array([x,y,1.0]))
        m = -l[0]/l[1]
        b = -l[2]/l[1]
        # equation of the line is y = m*x+b
        y_for_x_min = m*0.0+b
        y_for_x_max = m*(im.shape[1]/2.0-1)+b
        # plot the epipolar line
        cv2.line(image_to_display,(0,int(y_for_x_min)),(int(image_to_display.shape[1]-1.0),int(y_for_x_max)),0)
    cv2.imshow("RECT_IMAGE", image_to_display)

    return best_pcloud, depths, im1_pts, im2_pts

def set_corner_threshold(thresh):
    """ Sets the threshold to consider an interest point a corner.  The higher the value
        the more the point must look like a corner to be considered """
    global corner_threshold
    corner_threshold = thresh/1000.0

def set_ratio_threshold(thresh):
    """ Sets the ratio of the nearest to the second nearest neighbor to consider the match a good one """
    global ratio_threshold
    ratio_threshold = thresh/100.0

def set_epipolar_threshold(thresh):
    """ Sets the maximum allowable epipolar error to be considered an inlier by RANSAC """
    global epipolar_threshold
    epipolar_threshold = exp(-10+thresh/10.0)

def get_camera_info(msg):
    global K
    global D
    K = np.asarray(msg.K).reshape((3,3))
    D = np.asarray(msg.D)

def process_image(msg):
    global images
    global timestamps
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    images.append(gray_image)
    timestamps.append(msg.header.stamp)
    if len(images) > 100:
        images = images[1:]
        timestamps = timestamps[1:]

def process_image_rect(msg):
    global rect_images
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    rect_images.append(image)
    if len(rect_images) > 100:
        rect_images = rect_images[1:]

if __name__ == '__main__':
    K = None
    D = None
    images = []
    timestamps = []
    rect_images = []
    bridge = CvBridge()
    rospy.init_node('depth_tracker')

    rospy.Subscriber('/camera/camera_info', CameraInfo, get_camera_info)
    rospy.Subscriber('/camera/image_raw', Image, process_image)
    rospy.Subscriber('/camera/image_rect', Image, process_image_rect)

    if cv2.__version__=='3.1.0-dev':
        detector = cv2.xfeatures2d.SIFT_create()
        extractor = detector
    else:
        detector = cv2.FeatureDetector_create('SIFT')
        extractor = cv2.DescriptorExtractor_create('SIFT')

    matcher = cv2.BFMatcher()

    cv2.namedWindow("MYWIN")

    # create a simple UI for setting corner and ratio thresholds
    cv2.namedWindow('UI')
    cv2.createTrackbar('Corner Threshold', 'UI', 0, 100, set_corner_threshold)
    cv2.createTrackbar('Ratio Threshold', 'UI', 100, 100, set_ratio_threshold)
    cv2.createTrackbar('Epipolar Error Threshold', 'UI', 50, 100, set_epipolar_threshold)

    while not rospy.is_shutdown():
        # compute the 3-d coordinates and scaled depths for visualization
        compute_depths()
        key = cv2.waitKey(30)

    cv2.destroyAllWindows()
