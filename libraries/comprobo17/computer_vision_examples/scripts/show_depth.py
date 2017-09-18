#!/usr/bin/env python

""" Plot the depth of corresponding points.  In this case the points were identified by hand selecting
    correspondences """

import cv2
import pickle
import numpy as np
import math
import rospy
import rospkg

# These are the calibration parameters that are appropriate to the images we are using
D = np.array( [0.08683, -0.28966000000000003, -0.00045000000000000004, -0.00015000000000000001, 0.0])
K = np.array( [[651.38582, 0.0, 327.26766], [0.0, 650.2441, 242.38098],[ 0.0, 0.0, 1.0]])

W = np.array([[0.0, -1.0, 0.0],
              [1.0, 0.0, 0.0],
              [0.0, 0.0, 1.0]])

def triangulate_points(pt_set1, pt_set2, P, P1):
    """ Triangulate points using optimal trignaulation """
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
    """ Test how well two points fit the epipolar constraint using
        the formula pt1'*E*pt2 """
    pt1_h = np.zeros((3,1))
    pt2_h = np.zeros((3,1))
    pt1_h[0:2,0] = pt1.T
    pt2_h[0:2,0] = pt2.T
    pt1_h[2] = 1.0
    pt2_h[2] = 1.0
    return pt2_h.T.dot(E).dot(pt1_h)

def test_triangulation(P,pcloud):
    """ Compute the proportion of points that fall infront of the image plane (z > 0) """
    P4x4 = np.eye(4)
    P4x4[0:3,:] = P
    pcloud_3d = pcloud[:,0:3]
    projected = cv2.perspectiveTransform(np.array([pcloud_3d]),P4x4)
    return np.mean(projected[0,:,2]>0.0)

def mouse_event(event,x,y,flag,im):
    """ Plot an epipolar line when a click occurs in the left image """
    if event == cv2.EVENT_FLAG_LBUTTON:
        if x < im.shape[1]/2.0:
            l = F.dot(np.array([x,y,1.0]))
            m = -l[0]/l[1]
            b = -l[2]/l[1]
            # equation of the line is y = m*x+b
            y_for_x_min = m*0.0+b
            y_for_x_max = m*(im.shape[1]/2.0-1)+b
            # plot the epipolar line
            cv2.line(im,(int(im.shape[1]/2.0),int(y_for_x_min)),(int(im.shape[1]-1.0),int(y_for_x_max)),(255,0,0))

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    im1_file = rospack.get_path('computer_vision_examples') + '/images/frame0000.jpg'
    im2_file = rospack.get_path('computer_vision_examples') + '/images/frame0001.jpg'

    im1 = cv2.imread(im1_file)
    im2 = cv2.imread(im2_file)

    correspondences_file = rospack.get_path('computer_vision_examples') + '/images/correspondences.pickle'
    f = open(correspondences_file,'r')
    correspondences = pickle.load(f)
    f.close()

    im1_pts = np.zeros((len(correspondences[0]),2))
    im2_pts = np.zeros((len(correspondences[1]),2))

    im = np.array(np.hstack((im1,im2)))

    # plot the points
    for i in range(len(im1_pts)):
        im1_pts[i,0] = correspondences[0][i][0]
        im1_pts[i,1] = correspondences[0][i][1]
        im2_pts[i,0] = correspondences[1][i][0]
        im2_pts[i,1] = correspondences[1][i][1]

        cv2.circle(im,(int(im1_pts[i,0]),int(im1_pts[i,1])),2,(255,0,0),2)
        cv2.circle(im,(int(im2_pts[i,0]+im1.shape[1]),int(im2_pts[i,1])),2,(255,0,0),2)

    im1_pts_augmented = np.zeros((1,im1_pts.shape[0],im1_pts.shape[1]))
    im1_pts_augmented[0,:,:] = im1_pts
    im2_pts_augmented = np.zeros((1,im2_pts.shape[0],im2_pts.shape[1]))
    im2_pts_augmented[0,:,:] = im2_pts

    im1_pts_ud = cv2.undistortPoints(im1_pts_augmented,K,D)
    im2_pts_ud = cv2.undistortPoints(im2_pts_augmented,K,D)

    E, mask = cv2.findFundamentalMat(im1_pts_ud,im2_pts_ud,cv2.FM_RANSAC)

    im1_pts_ud_fixed, im2_pts_ud_fixed = cv2.correctMatches(E, im1_pts_ud, im2_pts_ud)
    use_corrected_matches = True
    if not(use_corrected_matches):
        im1_pts_ud_fixed = im1_pts_ud
        im2_pts_ud_fixed = im2_pts_ud

    epipolar_error = np.zeros((im1_pts_ud_fixed.shape[1],))
    for i in range(im1_pts_ud_fixed.shape[1]):
        epipolar_error[i] = test_epipolar(E,im1_pts_ud_fixed[0,i,:],im2_pts_ud_fixed[0,i,:])

    # calculate F since we know K
    F = np.linalg.inv(K.T).dot(E).dot(np.linalg.inv(K))
    U, Sigma, V = np.linalg.svd(E)

    # these are the two possible rotation matrices
    R1 = U.dot(W).dot(V)
    R2 = U.dot(W.T).dot(V)

    if np.linalg.det(R1)+1.0 < 10**-8:
        # if we accidentally got a rotation matrix with a negative determinant,
        # flip sign of E and recompute everything
        E = -E
        F = np.linalg.inv(K.T).dot(E).dot(np.linalg.inv(K))
        U, Sigma, V = np.linalg.svd(E)

        R1 = U.dot(W).dot(V)
        R2 = U.dot(W.T).dot(V)

    # these are the two translations that are possible
    t1 = U[:,2]
    t2 = -U[:,2]

    P = np.array([[1.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.0]]);
    P1_possibilities = []
    P1_possibilities.append(np.column_stack((R1,t1)))
    P1_possibilities.append(np.column_stack((R1,t2)))
    P1_possibilities.append(np.column_stack((R2,t1)))
    P1_possibilities.append(np.column_stack((R2,t2)))

    pclouds = []
    for P1 in P1_possibilities:
        pclouds.append(triangulate_points(im1_pts_ud_fixed, im2_pts_ud_fixed, P, P1))

    # compute the proportion of points that fall in front of the image plane so we can narrow it down to
    # one possible rotation and translation
    infront_of_camera = []
    for i in range(len(P1_possibilities)):
        infront_of_camera.append(test_triangulation(P,pclouds[i])+test_triangulation(P1_possibilities[i],pclouds[i]))

    best_pcloud = pclouds[np.argmax(infront_of_camera)]
    depths = best_pcloud[:,2] - min(best_pcloud[:,2])
    depths = depths / max(depths)

    for i in range(best_pcloud.shape[0]):
        cv2.circle(im,(int(im1_pts[i,0]),int(im1_pts[i,1])),int(max(1.0,depths[i]*20.0)),(0,255,0),1)

    cv2.imshow("MYWIN",im)
    cv2.setMouseCallback("MYWIN",mouse_event,im)
    while not rospy.is_shutdown():
        cv2.imshow("MYWIN",im)
        cv2.waitKey(50)
    cv2.destroyAllWindows()
