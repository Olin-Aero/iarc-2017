#!/usr/bin/env python2

"""
Gazebo Ground Truth Interface.
Intended as placeholder until actual roomba sighting code works.
"""

import numpy as np
import rospy
import tf

from sensor_msgs.msg import CameraInfo
from gazebo_msgs.msg import ModelStates
from iarc_main.msg import Roomba, RoombaList
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, Point, PointStamped, PoseStamped

from f_utils import observability, PolygonObservation

R = 3.0

def is_visible(src, dst):
    # dst pose is visible from src pose
    p0 = src.position
    p1 = dst.position
    r2 = (p0.x - p1.x)**2 + (p0.y - p1.y)**2
    return (r2 < R**2) # within +-3m radius

class GazeboInterface(object):
    def __init__(self):
        rospy.init_node("gz_interface")
        self._tf = tf.TransformListener()

        self._cam_frame = None
        self._cam_sub = rospy.Subscriber('/ardrone/bottom/camera_info', CameraInfo, self._cam_cb, queue_size=1)

        self._sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self._gz_cb)
        self._pub = rospy.Publisher('visible_roombas', RoombaList, queue_size=1)
        self._msg = None

    def _cam_cb(self, msg):
        """ Get camera info once, and quit """
        self._K = np.reshape(msg.K, (3,3))
        self._w = msg.width
        self._h = msg.height
        self._cam_frame = msg.header.frame_id

        if self._cam_sub:
            self._cam_sub.unregister()
            self._cam_sub = None

    def _gz_cb(self, msg):
        # save msg for later proc ...
        self._msg = msg

    def publish(self):
        msg = self._msg
        self._msg = None
        if msg is None:
            return

        now = rospy.Time.now()
        #cov = np.zeros((6,6)) #???
        cov = 1e-6 * np.eye(6)
        cov = cov.flatten()

        # names & poses
        n = len(msg.name)
        qi = msg.name.index("quadrotor")
        p0 = msg.pose[qi]

        obs = []

        # visibility
        try:
            if self._cam_frame is not None:
                _, q = self._tf.lookupTransform(self._cam_frame, 'map', rospy.Time(0))
                t, _ = self._tf.lookupTransform('map', self._cam_frame, rospy.Time(0))
                q = [q[3], q[0], q[1], q[2]] # reorder xyzw-> wxyz
                ar = observability(self._K, self._w, self._h, q, t, False)
                obs_ar = PolygonObservation(ar)
            else:
                return
        except Exception as e:
            print 'ex1', e
            return

        for n, p in zip(msg.name, msg.pose):
            if (n == 'quadrotor'):
                continue

            if not ([p.position.x, p.position.y] in obs_ar):
                continue

            # pretend like the drone saw it
            p = PoseStamped(Header(frame_id="map", stamp=rospy.Time()), p) # gz in map frame
            try:
                p = self._tf.transformPose("/drone/base_footprint", p) # to drone frame
            except Exception:
                print '??'
                break

            loc = PoseWithCovarianceStamped(
                    p.header,
                    PoseWithCovariance(
                        p.pose,
                        cov
                    ))

            if (n.startswith('target')):
                #TODO(yoonyoungcho) : Green-Red Differentiation somehow?
                obs.append(Roomba(last_seen=now, frame_id="/drone/base_footprint", type=Roomba.GREEN, visible_location=loc))
            elif (n.startswith('obstacle')):
                obs.append(Roomba(last_seen=now, frame_id="/drone/base_footprint", type=Roomba.OBSTACLE, visible_location=loc))

        msg = RoombaList(
                header = Header(stamp=now, frame_id="/drone/base_footprint"),
                data = obs
                )
        self._pub.publish(msg)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

def main():
    gi = GazeboInterface()
    gi.run()

if __name__ == "__main__":
    main()
