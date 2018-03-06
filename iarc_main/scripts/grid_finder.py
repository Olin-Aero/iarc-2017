#!/usr/bin/env python

''' 
Subscribes to a camera feed and publishes the pose of a grid square.
Image coordinate system: z is into image, x is right, y is down.
Global coordinate system: x is forward, y is left, z is up.

rosparam set use_sim_time true
rosbag play --clock --rate 0.2 --start 10 filename.something
'''

import numpy as np
import cv2
import rospy
import tf
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from tf.transformations import *

IMAGE_FEED = "/ardrone/bottom/image_raw/compressed" # ROS topic publishing grid images
SENSOR_FEED = "/odometry/filtered" # ROS topic publishing sensor data
LOWER_COLOR_THRESHOLD = 130#150 # Darkest color of a line out of 255
UPPER_COLOR_THRESHOLD = 255 # Lightest color of a line out of 255
MIN_CONTOUR_SIZE = 200#200 # Size cutoff in pixels for filtering out noise
DILATION_KERNEL_SIZE = 5 # Size of kernel in sqrt(pixels) for dilation
DILATION_STRENGTH = 7 # Number of iterations to apply the dilation operation
BORDER_SIZE = 5 # Size of border in pixels to ignore when finding lines
HOUGH_DIST_RES = 1 # Distance resolution of Hugh line transform in pixels
HOUGH_ANGLE_RES = np.pi/180 # Angular resolution of Hugh line transform in radians
HOUGH_THRESHOLD = 85#100 # Minimum number of points needed to determine a line
MIN_DIST = 100#50 # Distance in pixels between 2 lines to be considered different
MIN_ANGLE = np.pi/12 # Angle in radians between 2 lines to be considered different
MIN_INTERSECT_ANGLE = np.pi/12 # Minimum angle in radians between 2 intersecting lines
SIDE_LENGTH = 1.0 # Size of original grid square in meters
CAMERA_RATIO = 4/3*720 #4/3*720 #3264 # Intrinsic property of camera
TIME_OFFSET = 0 # Amount to project timestamp forward in time
SAMPLE_PERIOD = 1 # Number of frames between samples

class grid_finder:
    def __init__(self):
        rospy.init_node('grid_finder')
        self.count = -1

        self._odom_frame = rospy.get_param('~odom', default='odom')
        self._ci_topic = rospy.get_param('~camera_info', default='/ardrone/bottom/camera_info')

        # conversion utilities
        self._cv = CvBridge()
        self._tf = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # initialize data
        self.odomGridPose = tf.transformations.euler_matrix(0,0,0)
        self.camOdomPose = None
        self.msg = None
        rospy.Subscriber(IMAGE_FEED, CompressedImage, self.image_raw_callback)

        # display annotations - useful for debugging.
        self._ann_out = rospy.get_param('~ann_out', default='') 
        if self._ann_out:
            self._ann_pub = rospy.Publisher(self._ann_out, Image)
            self._ann_img = None

        # get camera info
        self._K = None
        self._w = None
        self._h = None
        self._cam_frame = None
        self._ci_sub = rospy.Subscriber(self._ci_topic, CameraInfo, self._ci_cb)

        # mah
        self._x0 = [0., 0., 0.]
        self._q0 = [1., 0., 0., 0.] #wxyz

    def _ci_cb(self, msg):
        """ Get camera info once, and quit """
        self._K = np.reshape(msg.K, (3, 3))
        self._w = msg.width
        self._h = msg.height
        self._cam_frame = msg.header.frame_id

        if self._ci_sub:
            self._ci_sub.unregister()
            self._ci_sub = None

    def image_raw_callback(self, msg):
        self.count+=1
        try:
            self.msg = msg
        except CvBridgeError as e:
            print(e)

    def process_image(self):
        if self.msg is None:
            return
        msg = self.msg
        self.msg = None
        try:
            frame = self._cv.compressed_imgmsg_to_cv2(msg, "bgr8")

            if self._K is None:
                # need accurate camera parameters
                return

            if self._ann_out:
                pose, self._ann_img = findGrid(frame, annotate=True, K=self._K)
            else:
                pose = findGrid(frame, annotate=False, K=self._K)
            self._posedbg = pose

            if pose is not None:
                _, rvec, tvec = pose
                if rvec is not None and tvec is not None:
                    ang = np.linalg.norm(rvec)
                    q = tf.transformations.quaternion_about_axis(ang, (rvec/ang).ravel())
                    pose = PoseStamped(header=Header(frame_id=self._cam_frame),
                            pose=Pose(position=Point(*tvec), orientation=Quaternion(*q)))
                    try:
                        # previous grid -> odom
                        x0, q0 = self._x0, self._q0 
                        h0 = tf.transformations.euler_from_quaternion(q0)[-1]

                        # grid -> odom -> base_link -> camera -> square
                        pose = self.listener.transformPose('grid', pose)
                        e_x, e_q = pose.pose.position, pose.pose.orientation

                        # heading from quaternion, around z axis
                        e_h = tf.transformations.euler_from_quaternion([e_q.w, e_q.x, e_q.y, e_q.z])[-1]
                        e_x = np.asarray([e_x.x, e_x.y, e_x.z])

                        # "snap to nearest 0.5"
                        gt_x = np.round(e_x/0.5)*0.5
                        # "snap to nearest pi/2"
                        gt_h = np.round(e_h/(np.pi/2))*(np.pi/2)

                        err_x = e_x - gt_x 
                        err_h = e_h - gt_h

                        # apply soft update on grid->odom tf
                        # TODO : arbitrary 

                        alpha = 0.1
                        x = x0 - alpha*err_x
                        h = h0 - alpha*err_h
                        q = tf.transformations.quaternion_about_axis(h, [0,0,1])

                        self._x0 = x
                        self._x0[2] = 0
                        self._q0 = q
                    except tf.Exception as e:
                        rospy.logerr_throttle(1.0, 'failed to transform pose and stuff : {}'.format(e))

            #if not pose[0] is None:
            #    euler = euler_from_matrix(pose)
            #    if np.linalg.norm([(euler[0]+2*np.pi)%(2*np.pi)-np.pi, (euler[1]+np.pi)%(2*np.pi)-np.pi]) > np.pi/4:
            #        print(euler)
            #        return
            #    camOdomPose = poseFromTransform(self.listener.lookupTransform(msg.header.frame_id, self._odom_frame, msg.header.stamp-rospy.Duration(.1)))
            #    # if self.camOdomPose is None:
            #    #     self.camOdomPose = camOdomPose
            #    # camOdomPose = self.camOdomPose#np.identity(4)
            #    lastPose = np.dot(self.odomGridPose, camOdomPose)
            #    pose = updateLocation(updateAngle(pose, lastPose), lastPose)
            #    print(int(pose[0][3]/SIDE_LENGTH), int(pose[1][3]/SIDE_LENGTH), int(pose[2][3]/SIDE_LENGTH), int(euler_from_matrix(pose)[2]*180/np.pi)%360)
            #    self.odomGridPose = np.dot(pose, np.linalg.inv(camOdomPose))

            self.publish()
        except CvBridgeError as e:
            print(e)

    def publish(self):
        if self._posedbg is not None:
            _, rvec, tvec = self._posedbg
            if rvec is not None and tvec is not None:
                # don't bother with Rodrigues(); straight to tf.
                h = np.linalg.norm(rvec)
                rvec /= h
                q = tf.transformations.quaternion_about_axis(h, rvec.ravel())
                self._tf.sendTransform(tvec, q, rospy.Time.now(), 'square', self._cam_frame)

        # publish tf ...
        #print self._x0
        #if self.odomGridPose is not None:
        #    pos = translation_from_matrix(self.odomGridPose)
        #    quat = quaternion_from_matrix(self.odomGridPose)
        #    self._tf.sendTransform(pos, quat, rospy.Time.now() + rospy.Duration(TIME_OFFSET), self._odom_frame, "grid")    
        self._tf.sendTransform(self._x0, self._q0, rospy.Time.now(), self._odom_frame, "grid")

        # publish annotation ...
        if self._ann_img is not None:
            try:
                ann_msg = self._cv.cv2_to_imgmsg(self._ann_img, "bgr8")
                self._ann_pub.publish(ann_msg)
            except CvBridgeError as e:
                rospy.logerr_throttle(1.0, 'Failed to publish annotations : {}'.format(e))
    
    def run(self):
        r = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            self.process_image()
            r.sleep()
        cv2.destroyAllWindows()

def findGrid(frame, annotate, K):
    '''Determines the location of the camera frame using a grid of tape'''

    # Save initial image
    orig = np.copy(frame)
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

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
    frame = cv2.erode(frame, kernel, iterations=1)

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
    pose = getPose(vertices, SIDE_LENGTH, shape, K)
    
    # Annotate image
    if annotate:
        ann = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if lines!=None:
            for line in lines:
                rho = line[0]
                theta = line[1]
                plotline = getLine(rho, theta)
                cv2.line(ann,plotline[0],plotline[1],(255,255,0),5)
        if intersections:
            for p in intersections:
                cv2.line(ann,p[0:2],p[0:2],(0,0,255),50)
        if vertices:
            cv2.line(ann,vertices[1][0:2],vertices[1][0:2],(0,255,0),70)
            cv2.line(ann,vertices[2][0:2],vertices[2][0:2],(0,255,255),60)
            cv2.line(ann,vertices[3][0:2],vertices[3][0:2],(255,0,255),50)

            cv2.line(ann, vertices[0][0:2], vertices[2][0:2], (255,0,0), 40)
            cv2.line(ann, vertices[1][0:2], vertices[3][0:2], (255,0,0), 40)
        if mid:
            cv2.line(ann,mid[0:2],mid[0:2],(255,255,0),100)

        ann = cv2.addWeighted(orig, 0.5, ann, 0.5, 0.0)
        return pose, ann
    else:
        return pose

def skeleton(img):
    ''' Converts an image into a morphological skeleton '''
    skeleton = cv2.bitwise_and(img, cv2.bitwise_not(img))
    for i in range(10):
        kernel = np.ones((DILATION_KERNEL_SIZE, DILATION_KERNEL_SIZE), np.uint8)
        temp = cv2.erode(img, kernel, iterations=1)
        temp = cv2.dilate(img, kernel, iterations=1)
        skeleton = cv2.bitwise_or(skeleton, cv2.bitwise_and(img, cv2.bitwise_not(temp)))
        img = cv2.erode(img, kernel, iterations=1)
        if not cv2.countNonZero(img):
            break
    return skeleton

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

def getPose(vertices, sideLength, shape, K):
    '''Determines the pose given the coordinates of the transformed square'''
    if vertices is None or len(vertices) < 4:
        return None, None, None
    square1x1 = [[-1, -1, 0],[1,-1, 0],[1, 1, 0],[-1, 1, 0]]
    square = np.float32([[b/2.0*sideLength for b in a] for a in square1x1])
    #cameraMatrix = np.float64([[CAMERA_RATIO,0,shape[1]/2],[0,CAMERA_RATIO,shape[0]/2],[0,0,1]])
    cameraMatrix = K
    ret, rvec, tvec = cv2.solvePnP(square, np.float32(vertices), cameraMatrix, np.zeros(4))

    return ret, rvec, tvec

    #rmat = cv2.Rodrigues(rvec)[0] # grid in camera frame
    #rmat = np.transpose(rmat) # camera in grid frame
    #rot = [[1,0,0],[0,-1,0],[0,0,-1]] # camera to baselink
    #rmat = np.dot(rot,rmat) # baselink in grid frame
    #print(tvec)
    #
    ## x, y are flipped, but z correct
    #tvec = np.dot(rmat,-tvec) # change of basis vectors, tvec switched frames
    ## tvec = np.multiply(tvec, [1, 1, -1]) # baselink in grid frame
    ## Right way
    ## tvec = grid in cam frame in cam vectors
    ## -tvec = cam in grid frame in cam vectors
    ## tvec[2] *= -1 converts to cam in grid frame in baselink vectors = baselink/grid in baselink vectors
    ## rmat*tvec = baselink/grid in grid vectors

    ## pos = [-tvec[0][0],-tvec[1][0],tvec[2][0]]
    #pos = [tvec[0][0],tvec[1][0],tvec[2][0]]
    #pose = [np.append(rmat[0],pos[0]), np.append(rmat[1],pos[1]), np.append(rmat[2],pos[2]), [0,0,0,1]]

    #return pose

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

def updateAngle(pose, lastPose):
    '''Computes updated orientation, compensating for 90 degree rotations of grid squares'''
    yaw = euler_from_matrix(pose)[2]
    lastyaw = euler_from_matrix(lastPose)[2]
    delta = (yaw-lastyaw)%(np.pi/2)
    if delta > np.pi/4:
        delta -= np.pi/2
    delta -= yaw - lastyaw
    # delta = (yaw-lastyaw + np.pi/4) % (np.pi/2) - yaw
    pose = np.dot(euler_matrix(0, 0, delta), pose)
    return pose

def updateLocation(pose, lastPose):
    '''Computes updated global position, compensating for switching between grid squares'''
    pos = translation_from_matrix(pose)
    rot = euler_matrix(*decompose_matrix(np.dot(pose, np.linalg.inv(lastPose)))[2])
    lastPose = np.dot(rot, lastPose)
    lastPos = translation_from_matrix(lastPose)
    dX = (pos[0] - lastPos[0])%SIDE_LENGTH
    dY = (pos[1] - lastPos[1])%SIDE_LENGTH
    if dX > SIDE_LENGTH/2:
        dX -= SIDE_LENGTH
    if dY > SIDE_LENGTH/2:
        dY -= SIDE_LENGTH
    dX -= pos[0] - lastPos[0]
    dY -= pos[1] - lastPos[1]
    pose = np.dot(translation_matrix([dX, dY, 0]), pose)
    return pose

def poseFromTransform(transform):
    '''Converts a TransformStamped to a 4x4 matrix'''
    pos = transform[0]
    quat = transform[1]
    pose = compose_matrix(angles=euler_from_quaternion(quat), translate=pos)
    return pose

if __name__ == '__main__':
    finder = grid_finder()
    finder.run()
