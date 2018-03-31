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
        self._decay_rate = rospy.get_param('~decay_rate', default=1e-2)
        self._map_size = rospy.get_param('~map_size', default=256)
        self._resolution = rospy.get_param('~resolution', default=0.078125) #256px->20.0m
        self._rate = rospy.get_param('~rate', default=50.0)
        self._ex = Explorer(self._map_size, self._decay_rate, 3.0)

        # for self-motion
        self._tf = tf.TransformListener(True, cache_time=rospy.Duration(5))

        # for roombas
        self._rsub = rospy.Subscriber(self._roomba_topic,
                RoombaList, self.roomba_cb)

    def roomba_cb(self, msg):
        pass

    def run(self):
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            self
            rate.sleep()

def main():
    n=256
    decay=1e-2
    radius=5
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
    rospy.init_node('explorer')
    explorer = ExplorerROS()
    explorer.run()

if __name__ == "__main__":
    main()
