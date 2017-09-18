#!/usr/bin/env python

""" A meanshift tracker using color histograms """

import cv2
import pickle
import rospy
import numpy as np

class ObjectTracker(object):
	""" An object that encompasses the tracking functionality """
	SELECTING_QUERY_IMG = 0
	SELECTING_ROI_PT_1 = 1
	SELECTING_ROI_PT_2 = 2

	""" Object Tracker shows the basics of tracking an object based on keypoints """
	def __init__(self):
		self.query_img = None
		self.query_roi = None
		self.last_detection = None

		self.state = ObjectTracker.SELECTING_QUERY_IMG

	def get_query_histogram(self):
		""" set up the ROI for tracking """
		roi = self.query_img[self.query_roi[1]:self.query_roi[3],self.query_roi[0]:self.query_roi[2],:]
		hsv_roi =  cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
		# play with the number of histogram bins by changing histSize
		self.query_hist = cv2.calcHist([hsv_roi],[0],mask=None,histSize=[256],ranges=[0,255])
		cv2.normalize(self.query_hist,self.query_hist,0,255,cv2.NORM_MINMAX)

	def track(self,im):
		""" Track using histogram back projection """
		im_hsv = cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
		track_im = cv2.calcBackProject([im_hsv],[0],self.query_hist,[0,255],1)

		track_im_visualize = track_im.copy()
		# convert to (x,y,w,h)
		track_roi = (self.last_detection[0],self.last_detection[1],self.last_detection[2]-self.last_detection[0],self.last_detection[3]-self.last_detection[1])

		# Setup the termination criteria, either 10 iteration or move by atleast 1 pt
		# this is done to plot intermediate results of mean shift
		for max_iter in range(1,10):
			term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, max_iter, 1 )
			(ret, intermediate_roi) = cv2.meanShift(track_im,track_roi,term_crit)
			cv2.rectangle(track_im_visualize,(intermediate_roi[0],intermediate_roi[1]),(intermediate_roi[0]+intermediate_roi[2],intermediate_roi[1]+intermediate_roi[3]),max_iter/10.0,2)

		self.last_detection = [intermediate_roi[0],intermediate_roi[1],intermediate_roi[0]+intermediate_roi[2],intermediate_roi[1]+intermediate_roi[3]]
		cv2.imshow("track_win",track_im_visualize)

def mouse_event(event,x,y,flag,im):
	""" Handles the logic of selecting the region to track """
	if event == cv2.EVENT_FLAG_LBUTTON:
		if tracker.state == tracker.SELECTING_QUERY_IMG:
			tracker.query_img_visualize = frame.copy()
			tracker.query_img = frame
			tracker.query_roi = None
			tracker.state = tracker.SELECTING_ROI_PT_1
		elif tracker.state == tracker.SELECTING_ROI_PT_1:
			tracker.query_roi = [x,y,-1,-1]
			cv2.circle(tracker.query_img_visualize,(x,y),5,(255,0,0),5)
			tracker.state = tracker.SELECTING_ROI_PT_2
		else:
			tracker.query_roi[2:] = [x,y]
			tracker.last_detection = tracker.query_roi
			cv2.circle(tracker.query_img_visualize,(x,y),5,(255,0,0),5)
			tracker.get_query_histogram()
			tracker.state = tracker.SELECTING_QUERY_IMG

if __name__ == '__main__':
	tracker = ObjectTracker()
	cap = cv2.VideoCapture(0)
	cv2.namedWindow("MYWIN")
	cv2.setMouseCallback("MYWIN",mouse_event)

	while not rospy.is_shutdown():
		ret, frame = cap.read()
		frame = np.array(cv2.resize(frame,(frame.shape[1]/2,frame.shape[0]/2)))

		if tracker.state == tracker.SELECTING_QUERY_IMG:
			if tracker.query_roi != None:
				tracker.track(frame)

				# add the query image to the side
				combined_img = np.zeros((frame.shape[0],frame.shape[1]+(tracker.query_roi[2]-tracker.query_roi[0]),frame.shape[2]),dtype=frame.dtype)
				combined_img[:,0:frame.shape[1],:] = frame
				combined_img[0:(tracker.query_roi[3]-tracker.query_roi[1]),frame.shape[1]:,:] = (
						tracker.query_img[tracker.query_roi[1]:tracker.query_roi[3],
										  tracker.query_roi[0]:tracker.query_roi[2],:])

				cv2.rectangle(combined_img,(tracker.last_detection[0],tracker.last_detection[1]),(tracker.last_detection[2],tracker.last_detection[3]),(0,0,255),2)

				cv2.imshow("MYWIN",combined_img)
			else:
				cv2.imshow("MYWIN",frame)
		else:
			cv2.imshow("MYWIN",tracker.query_img_visualize)
		cv2.waitKey(50)
	cv2.destroyAllWindows()
