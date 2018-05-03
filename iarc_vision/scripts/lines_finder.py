#!/usr/bin/env python2
"""
Authored Yoonyoung Cho @ 03/29/2018

The objective of LinesFinder is to deal with thick-lines problem,
where a width of single line may not be presented as a single pixel,
and it may be difficult to pre-determine the thickness of the line.
This introduces an instability where each detection would be sensitive to erosion.
"""

from argparse import ArgumentParser
from abc import ABCMeta, abstractmethod
import cv2
import numpy as np

""" Utility Functions """
def l2rt(line):
    """ (x1,y1,x2,y2) -> (rho,theta) """
    x0,y0 = 0,0
    x1,y1,x2,y2 = line
    rho = -x2*y1 + y2*x1
    rho /= np.sqrt((x2-x1)**2 + (y2-y1)**2)
    theta = np.arctan2(x1-x2, y2-y1)
    return rho, theta

def rt2l(rt, scale=1000):
    """ (rho,theta) -> (x1,y1,x2,y2) """
    rho, theta = rt
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + scale*(-b))
    y1 = int(y0 + scale*(a))
    x2 = int(x0 - scale*(-b))
    y2 = int(y0 - scale*(a))
    return x1,y1,x2,y2

def rtclose(rt1, rt2,
        r_thresh = 10.0,
        t_thresh = np.deg2rad(5.0)
        ):
    """ compare lines in rho-theta space """
    r1,t1 = rt1
    r2,t2 = rt2
    return (abs(r1-r2)<r_thresh) and (abs(t1-t2) < t_thresh)

class LinesFinder(object):
    """ Defines Lines Finder Interface """
    __metaclass__ = ABCMeta
    """ Find Lines from Image """
    @abstractmethod
    def __call__(self, img):
        """ Binary Image -> List of Lines as [(x1,y1,x2,y2)] """
        pass

class HoughLinesFinder(LinesFinder):
    """ Find lines via cv2.HoughLines() """
    def __init__(self, rho, theta, threshold, ksize):
        self._rho = rho
        self._theta = theta
        self._threshold = threshold
        self._ksize = (ksize, ksize)
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ERODE, self._ksize)
        super(HoughLinesFinder, self).__init__()

    def merge(self, lines):
        """
        Average nearby lines in (rho, theta) coordinates.
        lines(rt) -> lines(rt)
        """
        if lines is None:
            return None

        lines2 = []
        while len(lines)>0:
            n = 1

            # reference
            l0 = lines.pop(0)
            r0, t0 = l0
            rTotal = l0[0]
            tTotal = l0[1]

            # compare + filter
            for line in lines:
                r, t = line
                if rtclose((r0,t0), (r,t), r_thresh=20.0):
                    n+=1
                    rTotal+=r
                    tTotal+=t
                    lines.remove(line)
            lines2+=[(rTotal/float(n), tTotal/float(n))]
        return lines2

    def __call__(self, img):
        """ Binary Image -> List of Lines as [(x1,y1,x2,y2)] """
        rho, theta = self._rho, self._theta
        threshold = self._threshold
        kernel = self._kernel

        img = cv2.erode(img, kernel, iterations=1)
        canny = cv2.Canny(img, 0, 255)
        lines = cv2.HoughLines(canny,rho,theta,threshold)
        if lines is None:
            return None

        lines = np.squeeze(lines, axis=1)
        lines = self.merge(lines.tolist())
        lines = [rt2l(rt) for rt in lines]
        return lines

class HoughLinesPFinder(LinesFinder):
    """ Find Lines via cv2.HoughLinesP() """
    def __init__(self, rho, theta,
            threshold,
            min_length,
            max_gap):
        self._rho = rho
        self._theta = theta
        self._threshold = threshold
        self._min_length = min_length
        self._max_gap = max_gap
        super(HoughLinesPFinder, self).__init__()

    def merge(self, lines):
        """
        Convert lines to rho-theta space,
        then merge neighboring lines.
        lines (xyxy) -> lines (xyxy)
        """
        n = len(lines)
        rts = [l2rt(l) for l in lines]
        skip = []
        res = []
        for i in range(n):
            rt_acc = [rts[i]]

            # check merged
            if i in skip:
                continue
            skip.append(i)

            # process remainder
            for j in range(i+1,n):
                # check merged
                if j in skip:
                    continue
                if rtclose(rts[i], rts[j]):
                    skip.append(j)
                    rt_acc.append(rts[j])

            rt_acc = np.asarray(rt_acc)

            # convert to line
            r = np.mean(rt_acc[:,0])

            # take cyclic mean, for robustness
            #t = np.mean(rt_acc[:,1])
            s = np.sum(np.sin(rt_acc[:,1]))
            c = np.sum(np.cos(rt_acc[:,1]))
            t = np.arctan2(s, c)

            res.append(rt2l((r,t)))
        return res

    def __call__(self, img):
        """ Binary Image -> List of Lines as [(x1,y1,x2,y2)] """
        rho, theta = self._rho, self._theta
        threshold = self._threshold
        min_length = self._min_length
        max_gap = self._max_gap
        lines = cv2.HoughLinesP(img,
                rho, theta, threshold,
                minLineLength=min_length,
                maxLineGap=max_gap)

        if lines is not None:
            lines = np.squeeze(lines, axis=1)
            lines = self.merge(lines)
            lines = [rt2l(l2rt(l)) for l in lines]

        return lines

def getmask(frame, lo=200, hi=255, min_ctr=200):
    orig = np.copy(frame)
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # Load image
    shape = frame.shape
    height = shape[0]
    width = shape[1]

    # Color filtering
    lower = np.array([lo])
    upper = np.array([hi])
    mask = cv2.inRange(frame, lower, upper)
    frame = cv2.bitwise_and(frame, frame, mask=mask)

    # TODO : this method fattens the lines.
    # @(superduperpacman) other methods to filter out small particles?
    # Filter out small particles
    #im2, contours, hierarchy = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #filter(lambda x:len(x)>min_ctr, contours)
    #cv2.drawContours(frame, contours, -1, 255, 3)
    return frame

class LinesApp(object):
    def __init__(self, img, scale=1.0, resolution=100, max_scale=2.0):
        self._img = img
        self._mask = getmask(img)
        self._img1 = img
        self._img2 = img
        self._scale = scale
        self._res = resolution
        self._max_scale = max_scale
        rint = lambda x : int(np.round(x))

        cv2.namedWindow('image')
        cv2.createTrackbar('scale', 'image', rint(self._res*self._scale),
                rint(self._res*self._max_scale), lambda s:self.proc(self.v2s(s)))

    def v2s(self, value):
        """ trackbar value --> scale """
        return float(value)/self._res

    def proc(self, scale):
        img = self._img
        print('Current Scale : {}'.format(scale))
        img = cv2.resize(img, (0,0), fx=scale, fy=scale)

        # convert to binary
        mask = getmask(img)
        #mask = cv2.inRange(img, 
        #        np.asarray([200,200,200]), 
        #        np.asarray([255,255,255]))
        h,w = img.shape[:2]

        # parametrization
        rho = 1.0 #distance resolution
        theta = np.deg2rad(1.0) #360
        threshold = 80 #???
        min_length = 0.25 * min(w,h)#pixels?
        max_gap = 10 #pixels?
        ksize = 5

        finder1 = HoughLinesFinder(rho, theta, threshold, ksize)
        finder2 = HoughLinesPFinder(rho, theta, threshold, min_length, max_gap)

        lines1 = finder1(mask)
        lines2 = finder2(mask)

        img1 = img.copy()
        img2 = img.copy()

        if lines1 is not None:
            # BLUE = HoughLinesFinder
            for line in lines1:
                x1,y1,x2,y2 = line
                cv2.line(img1, (x1,y1), (x2,y2), [255,0,0], 1)

        if lines2 is not None:
            # RED = HoughLinesPFinder
            for line in lines2:
                x1,y1,x2,y2 = line
                cv2.line(img2, (x1,y1), (x2,y2), [0,0,255], 1)

        self._mask = mask
        self._img1 = img1
        self._img2 = img2

    def run(self):
        # Log
        print("BLUE (img1) : HoughLinesFinder()")
        print("RED (img2) : HoughLinesPFinder()")
        self.proc(self._scale)

        while True:
            k = cv2.waitKey(10)
            if k == 27:
                break
            cv2.imshow('image', self._img)
            cv2.imshow('mask', self._mask)
            cv2.imshow('img1', self._img1)
            cv2.imshow('img2', self._img2)

def main():
    parser = ArgumentParser()
    parser.add_argument('file', type=str)
    args = parser.parse_args()

    img = cv2.imread(args.file)
    if img is None:
        print("Error : Specified Input File \"{}\" is invalid.".format(args.file))
        parser.print_usage()
    else:
        app = LinesApp(img)
        app.run()

if __name__ == "__main__":
    main()
