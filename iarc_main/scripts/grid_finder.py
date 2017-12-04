#!/usr/bin/env python

''' 
Subscribes to a camera feed and publishes the pose of a grid square.
Does not yet track change in squares to find global reference frame.
Image coordinate system: z is into image, x is right, y is down.
Global coordinate system: z is down, right hand rule

@author Paul Nadan
11/15/2017
'''

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from tf.transformations import quaternion_from_euler, euler_from_quaternion


IMAGE_FEED = "/usb_cam/image_raw" # ROS topic publishing grid images
LOWER_COLOR_THRESHOLD = 150 # Darkest color of a line out of 255
UPPER_COLOR_THRESHOLD = 255 # Lightest color of a line out of 255
MIN_CONTOUR_SIZE = 200 # Size cutoff in pixels for filtering out noise
DILATION_KERNEL_SIZE = 5 # Size of kernel in sqrt(pixels) for dilation
DILATION_STRENGTH = 1 # Number of iterations to apply the dilation operation
BORDER_SIZE = 5 # Size of border in pixels to ignore when finding lines
HOUGH_DIST_RES = 1 # Distance resolution of Hugh line transform in pixels
HOUGH_ANGLE_RES = np.pi/180 # Angular resolution of Hugh line transform in radians
HOUGH_THRESHOLD = 100 # Minimum number of points needed to determine a line
MIN_DIST = 50 # Distance in pixels between 2 lines to be considered different
MIN_ANGLE = np.pi/36 # Angle in radians between 2 lines to be considered different
MIN_INTERSECT_ANGLE = np.pi/12 # Minimum angle in radians between 2 intersecting lines
SIDE_LENGTH = 9 # Size of original grid square in inches
CAMERA_RATIO = 4/3*720 #3264 # Intrinsic property of camera
SAMPLE_PERIOD = 100 # Number of frames between samples

class grid_finder:
    def __init__(self):
        rospy.init_node('grid_finder')
        self.count = -1
        self.pub = rospy.Publisher('/grid', PoseStamped, queue_size=10)
        rospy.Subscriber(IMAGE_FEED, Image, self.image_raw_callback)
        self.bridge = CvBridge()
        self.x = 0
        self.y = 0
        self.angle = 0

    def image_raw_callback(self, msg):
        self.count+=1
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            pose = findGrid(frame,self.count%SAMPLE_PERIOD==0)
            # print(pose)
            if pose:
                angleOffset = self.updateAngle(pose)
                self.updateLocation(pose, angleOffset)
                output = PoseStamped()
                euler = np.array(euler_from_quaternion(pose[0]))
                euler[2] = self.angle
                output.pose.orientation = Quaternion(*quaternion_from_euler(*euler))
                output.pose.position.x = self.x
                output.pose.position.y = self.y
                print(int(self.x), int(self.y), int(pose[1][2]), int(self.angle*180/np.pi)%360)
                output.pose.position.z = pose[1][2]
                output.header.frame_id = "map"
                self.pub.publish(output)

        except CvBridgeError as e:
            print(e)

    def updateAngle(self, pose):
        '''Computes actual global yaw, compensating for 90 degree rotations of grid squares'''
        yaw = euler_from_quaternion(pose[0])[2]
        delta = (yaw-self.angle)%(np.pi/2)
        self.angle = self.angle+delta
        if delta > np.pi/4:
            self.angle = self.angle-np.pi/2
        return self.angle - yaw - np.pi/2
        # TODO: Gyro integration and filtering

    def updateLocation(self, pose, angleOffset=0):
        '''Computes actual global position, compensating for switching between grid squares'''
        # x2 = -pose[1][0]*np.cos(self.angle)-pose[1][1]*np.sin(self.angle)
        # y2 = -pose[1][0]*np.sin(self.angle)+pose[1][1]*np.cos(self.angle)
        x2 = np.cos(angleOffset)*pose[1][0]-np.sin(angleOffset)*pose[1][1]
        y2 = np.sin(angleOffset)*pose[1][0]+np.cos(angleOffset)*pose[1][1]
        dX = (x2 - self.x)%SIDE_LENGTH
        dY = (y2 - self.y)%SIDE_LENGTH
        self.x = self.x+dX
        self.y = self.y+dY
        if dX > SIDE_LENGTH/2:
            self.x = self.x - SIDE_LENGTH
        if dY > SIDE_LENGTH/2:
            self.y = self.y - SIDE_LENGTH
        # TODO: Optical flow integration and filtering
    
    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

def findGrid(frame, display):
    '''Determines the location of the camera frame using a grid of tape'''

    # Load image
    shape = frame.shape
    height = shape[0]
    width = shape[1]

    # Color filtering
    lower = np.array([LOWER_COLOR_THRESHOLD])
    upper = np.array([UPPER_COLOR_THRESHOLD])
    mask = cv2.inRange(frame, lower, upper)
    frame = cv2.bitwise_and(frame, frame, mask=mask)
            

    # Filter out small particles
    im2, contours, hierarchy = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    filter(lambda x:len(x)>MIN_CONTOUR_SIZE, contours)
    cv2.drawContours(frame, contours, -1, (0,255,0), 3)

    # Fill in gaps
    kernel = np.ones((DILATION_KERNEL_SIZE, DILATION_KERNEL_SIZE), np.uint8)
    frame = cv2.dilate(frame, kernel, iterations=1)

    # Find edges
    edges = cv2.Canny(frame,0,255)

    # Crop out the borders
    edges = edges[BORDER_SIZE:-BORDER_SIZE][BORDER_SIZE:-BORDER_SIZE]

    # Find and merge lines
    lines = cv2.HoughLines(edges,HOUGH_DIST_RES,HOUGH_ANGLE_RES,HOUGH_THRESHOLD)
    lines = mergeLines(lines)

    # Find intersections
    intersections = []
    if lines:
        for i, line1 in enumerate(lines):
            for line2 in lines[i:]:
                if abs(np.sin(line1[1]-line2[1]))>MIN_INTERSECT_ANGLE:
                    intersection = intersect(line1, line2)
                    if intersection:
                        intersections+=[intersection+(line1,line2)]

    # Find a grid square
    mid = midpoint(intersections, width, height)
    vertices = orderVertices(getVertices(mid, intersections))

    # Compute pose
    pose = getPose(vertices, SIDE_LENGTH)
    
    # Annotate image
    if display:
        color = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if lines!=None:
            for line in lines:
                rho = line[0]
                theta = line[1]
                plotline = getLine(rho, theta)
                cv2.line(color,plotline[0],plotline[1],(255,0,0),5)
        if intersections:
            for p in intersections:
                cv2.line(color,p[0:2],p[0:2],(0,0,255),50)
        if vertices:
            cv2.line(color,vertices[1][0:2],vertices[1][0:2],(0,255,0),70)
            cv2.line(color,vertices[2][0:2],vertices[2][0:2],(0,255,255),60)
            cv2.line(color,vertices[3][0:2],vertices[3][0:2],(255,0,255),50)
        if mid:
            cv2.line(color,mid[0:2],mid[0:2],(255,255,0),100)

        cv2.imshow("Grid Tracking", color)
        cv2.waitKey(3)

    return pose

def getLine(rho, theta):
    '''Convert (rho, theta) defined line to endpoints'''
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 10000*(-b))
    y1 = int(y0 + 10000*(a))
    x2 = int(x0 - 10000*(-b))
    y2 = int(y0 - 10000*(a))
    return ((x1,y1),(x2,y2))

def intersect(line1, line2):
    '''Find intersection of 2 lines in (rho, theta) coordinates'''
    r1 = line1[0]
    r2 = line2[0]
    t1 = line1[1]
    t2 = line2[1]
    if t1==t2:
        return
    if t1%(np.pi/2) == 0:
        t1 += .001
    if t2%(np.pi/2) == 0:
        t2 += .001
    y = (r2/np.sin(t2)-r1*np.cos(t2)/(np.sin(t2)*np.cos(t1)))/(1-np.tan(t1)/np.tan(t2))
    x = (r1-y*np.sin(t1))/np.cos(t1)
    return (int(x),int(y))

def mergeLines(lines):
    '''Average nearby lines in (rho, theta) coordinates'''
    if lines is None:
        return None
    lines = lines.tolist()
    lines2 = []
    while len(lines)>0:
        n = 1
        rTotal = lines[0][0][0]
        tTotal = lines[0][0][1]
        for line in lines[1:]:
            if abs(line[0][0]-lines[0][0][0]) < MIN_DIST:
                if abs(np.sin(line[0][1]-lines[0][0][1]))<MIN_ANGLE:
                    n+=1
                    rTotal+=line[0][0]
                    tTotal+=line[0][1]
                    lines.remove(line)
        del(lines[0])
        lines2+=[(rTotal/n, tTotal/n)]
    return lines2

def midpoint(points, w, h):
    '''Returns the center-most point'''
    best = None
    bestDist = w**2+h**2
    for p in points:
        if (p[0]-w/2)**2+(p[1]-h/2)**2 < bestDist:
            best = p
            bestDist = (p[0]-w/2)**2+(p[1]-h/2)**2
    return best

def getVertices(mid, points):
    '''Finds a quadrilateral with midpoint as a vertex'''
    if not mid or points is None or len(points) < 3:
        return None
    # Find point 1
    d1 = mid[2][1]
    p1 = nextPoint(mid, points, d1)
    if not p1:
        d1 = np.pi+d1
        p1 = nextPoint(mid, points, d1)

    # Find point 2
    d2 = mid[3][1]
    p2 = nextPoint(mid, points, d2)
    if not p2:
        d2 = np.pi+d2
        p2 = nextPoint(mid, points, d2)
    if not (p1 and p2):
        return None

    # Find point 3
    d3 = closestDirection(p1[2][1], p1[3][1], d2)
    p3 = nextPoint(p1, points, d3)
    if not p3:
        d3 = closestDirection(p2[2][1], p2[3][1], d1)
        p3 = nextPoint(p2, points, d3)
    if not p3:
        p3 = (p2[0]+p1[0]-mid[0],p2[1]+p1[1]-mid[1])
    return (mid[:2], p1[:2], p2[:2], p3[:2])

def nextPoint(p1, points, direction):
    '''Finds the closest point to p1 in the given direction'''
    direction = np.pi/2+direction # Change coordinate system
    d1 = (np.cos(direction), np.sin(direction))
    bestDist = 100000
    best = None
    for p2 in points:
        d2 = (p2[0]-p1[0], p2[1]-p1[1])
        dist = np.linalg.norm(d2)
        if dist > MIN_DIST and np.dot(d1[:2],d2[:2])/dist > 0.9:
            if dist < bestDist:
                bestDist = dist
                best = p2
    return best

def closestDirection(d1,d2,direction):
    '''Determines which of +-d1 and +-d2 is closest to direction'''
    v1 = (np.cos(direction), np.sin(direction))
    d = [d1, np.pi+d1, d2, np.pi+d2]
    v = [(np.cos(dn), np.sin(dn)) for dn in d]
    score = [np.dot(v1,v2)/np.linalg.norm(v2) for v2 in v]
    return d[score.index(max(score))]

def getPose(vertices, sideLength):
    '''Determines the pose given the coordinates of the transformed square'''
    if vertices is None or len(vertices) < 4:
        return None
    square1x1 = [[-1, -1, 0],[1,-1, 0],[1, 1, 0],[-1, 1, 0]]
    square = np.float32([[b/2.0*sideLength for b in a] for a in square1x1])
    cameraMatrix = np.float64([[CAMERA_RATIO,0,0],[0,CAMERA_RATIO,0],[0,0,1]])
    ret, rvec, tvec = cv2.solvePnP(square, np.float32(vertices), cameraMatrix, np.zeros(4))

    rmat = cv2.Rodrigues(rvec)[0]
    rmat = np.transpose(rmat)
    rot = [[1,0,0],[0,-1,0],[0,0,-1]]
    rmat = np.dot(rot,rmat)
    tvec = -np.dot(rmat,tvec)
    qw = np.sqrt(1 + rmat[0][0] + rmat[1][1] + rmat[2][2]) /2
    qx = (rmat[2][1] - rmat[1][2])/( 4 *qw)
    qy = (rmat[0][2] - rmat[2][0])/( 4 *qw)
    qz = (rmat[1][0] - rmat[0][1])/( 4 *qw)

    # angle = np.linalg.norm(rvec) # World to camera
    # angle = angle # Camera to world
    # axis = [x[0]/angle for x in rvec]
    # qx = axis[0] * np.sin(angle/2)
    # qy = axis[1] * np.sin(angle/2)
    # qz = axis[2] * np.sin(angle/2)
    # qw = np.cos(angle/2)
    # pos = [x[0] for x in tvec]
    
    quat = [qx,qy,qz,qw]
    pos = [tvec[0][0],tvec[1][0],tvec[2][0]]
    return (quat,pos)

def orderVertices(vertices):
    '''Returns the given vertices in counterclockwise order (relative to right-handed xy axes)'''
    if vertices is None or len(vertices)<4:
        return
    v1 = [a-b for a,b in zip(vertices[1],vertices[0])]
    v2 = [a-b for a,b in zip(vertices[2],vertices[0])]
    ccw = np.cross(v1,v2) > 0
    if ccw:
        order = [0,1,3,2]
    else:
        order = [0,2,3,1]
    return [vertices[i] for i in order]

if __name__ == '__main__':
    finder = grid_finder()
    finder.run()