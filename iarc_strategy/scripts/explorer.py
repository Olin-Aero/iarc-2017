#!/usr/bin/env python2
"""
Implements a simple exploration server,
based on past visited areas and TODO@(yoonyoungcho): roomba information.
"""
import numpy as np
import rospy
import tf
import cv2
#import imageio
from matplotlib import pyplot as plt

from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Quaternion, Transform, TransformStamped
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from iarc_main.msg import Roomba, RoombaList
from iarc_strategy.srv import ExplorationTarget, ExplorationTargetResponse

class Explorer(object):
    """ Simple Information-Search Explorer """
    def __init__(self, n, decay, radius):
        self._n = n
        self._map = np.zeros((n,n), dtype=np.float32)
        self._decay = decay
        self._radius = radius
    def update(self, pts, dt):
        """ Apply map decay and add information"""
        # apply decay
        self._map *= np.exp(-self._decay*dt)
        # update points
        # TODO : incorporate observation probabilities, etc.
        pts = np.int32(pts)
        for pt in pts:
            cv2.circle(self._map, tuple(pt[::-1]), self._radius, 1.0, -1)
    def target(self):
        """ Select Good target location based on quadrants """
        i0, j0 = 0, 0
        n = self._n

        while n > 1:
            n2 = n/2
            q_ul = np.sum(self._map[i0+0:i0+n2, j0+0:j0+n2])
            q_ur = np.sum(self._map[i0+0:i0+n2, j0+n2:j0+n])
            q_dl = np.sum(self._map[i0+n2:i0+n, j0+0:j0+n2])
            q_dr = np.sum(self._map[i0+n2:i0+n, j0+n2:j0+n])

            q = np.float32([q_ul, q_ur, q_dl, q_dr])
            
            #print 'q', q
            dis = [0,0,n2,n2]
            djs = [0,n2,0,n2]
            sel = np.random.choice(np.where(q == q.min())[0])
            i0, j0 = i0+dis[sel], j0+djs[sel]
            n=n2
        return i0, j0

class ExplorerROS(object):
    def __init__(self):
        self._roomba_topic = rospy.get_param('~roomba_topic', default='seen_roombas')
        self._decay_rate = rospy.get_param('~decay_rate', default=1e-1) #0.1 in ~20 sec.
        self._map_size = rospy.get_param('~map_size', default=256)
        self._resolution = rospy.get_param('~resolution', default=0.078125) #256px->20.0m
        self._rate = rospy.get_param('~rate', default=50.0)
        self._map_frame = rospy.get_param('~map_frame', default='map')
        self._drone_frame = rospy.get_param('~drone_frame', default='base_link')
        self._viz = rospy.get_param('~viz', default=False) # enable visualization
        # TODO : publish visualization info?
        # radius = 1.5m / self._resolution, observes +-1.5m around drone
        self._ex = Explorer(self._map_size, self._decay_rate, self.m2px(1.5, center=False))

        # for self-motion
        self._tf = tf.TransformListener(True, cache_time=rospy.Duration(5))

        # for roombas
        self._rsub = rospy.Subscriber(self._roomba_topic,
                RoombaList, self.roomba_cb)
        self._srv = rospy.Service('~explore', ExplorationTarget, self.target_cb)

    def m2px(self, x, center=False):
        """ meters -> pixels """
        res = np.divide(x, self._resolution)
        if center:
            res += self._map_size/2.0
        return res.astype(np.int32)

    def px2m(self, x, center=False):
        """ pixels -> meters """
        if center:
            x = np.subtract(x, self._map_size / 2.0)
        res = np.multiply(x, self._resolution)
        return res

    def target_cb(self, req):
        """ Fulfills ExplorationTarget Request """
        target = self._ex.target()
        target = self.px2m(target, center=True)
        res = ExplorationTargetResponse()
        res.target.x = target[0]
        res.target.y = target[1]
        res.target.z = 2.5 # TODO : arbitrary-ish height
        res.success = True
        return res

    def roomba_cb(self, msg):
        """ Update Map based on Roomba Information"""
        if not msg.header.frame_id == self._map_frame:
            rospy.logerr_throttle(0.5, "Incoming RoombaList() must be in ExplorerRos.map_frame !!")
            return

        # extract x,y position
        pts = [r.visible_location.pose.pose.position for r in msg.data]
        pts = [(pt.x, pt.y) for pt in pts]
        pts = self.m2px(pts, center=True)
        self._ex.update(pts, 0.0)

    def run(self):
        """ Run Explorer Sensing + Service """
        rate = rospy.Rate(self._rate)
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            # get time ...
            t1 = rospy.Time.now().to_sec()
            dt = t1-t0
            t0 = t1

            # update map ...
            pts = []
            try:
                pos, _ = self._tf.lookupTransform(self._map_frame, self._drone_frame, rospy.Time(0))
                #print pos
                pts.append(pos[:2])
            except tf.Exception as e:
                rospy.loginfo_throttle(1.0, "TF Lookup Failed: {}".format(e))
            pts = self.m2px(pts, center=True)
            self._ex.update(pts, dt)
            if self._viz:
                cv2.imshow('viz', self._ex._map)
                cv2.waitKey(10)
            rate.sleep()

def main():
    """ Test Explorer() without ROS Binding"""
    n=256
    decay=1e-2
    radius=20
    dt=0.1
    pos=np.float32([n/2, n/2])
    goal=None
    ex = Explorer(n=n, decay=decay, radius=radius)

    t = 0.0
    ds = []

    #with imageio.get_writer('explore.gif', mode='I',
    #        fps=60, palettesize=16, subrectangles=True) as writer:
    while True:
        density = 100 * np.mean(ex._map)
        print('Map Density : {:.2f} %'.format(density))
        ds.append(density)

        #print pos, goal
        viz = (255*ex._map).astype(np.uint8)
        cv2.putText(viz, 'Time : {}s'.format(t),
                (0, 250), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, 255, 2)
        cv2.imshow('map', viz)
        #writer.append_data(viz)
        ex.update([pos], dt)
        if goal is None:
            goal = ex.target()
        else:
            delta = np.subtract(goal, pos)
            dist = np.linalg.norm(delta)
            if dist < 4.0: #within +-4px
                goal = None
            else:
                # vel = 20 px / sec (=2px/it)
                pos += (50/dist*dt)*delta
        k = cv2.waitKey(10)
        t += dt
        if k == 27:
            break

    #plt.plot(dt*np.arange(len(ds)), ds)
    #plt.title('Map Density Over Time')
    #plt.xlabel('Time (s)')
    #plt.ylabel('Density (%)')
    #plt.grid()
    #plt.show()

def rosmain():
    """ Explorer Service """
    rospy.init_node('explorer')
    explorer = ExplorerROS()
    explorer.run()

if __name__ == "__main__":
    rosmain()
