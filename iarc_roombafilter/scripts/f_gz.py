#!/usr/bin/env python2

"""
Gazebo Ground Truth Interface.
Intended as placeholder until actual roomba sighting code works.
"""

import numpy as np
import rospy
import tf

from gazebo_msgs.msg import ModelStates
from iarc_main.msg import Roomba, RoombaSighting
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, Point, PointStamped, PoseStamped

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
        self._sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self._gz_cb)
        self._pub = rospy.Publisher('roomba_obs', RoombaSighting, queue_size=1)
        self._msg = None

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
        for n,p in zip(msg.name, msg.pose):
            if (n == 'quadrotor'):
                continue
            if not is_visible(p0, p):
                continue

            # pretend like the drone saw it
            p = PoseStamped(Header(frame_id="map", stamp=rospy.Time()), p) # gz in map frame
            try:
                p = self._tf.transformPose("/drone/base_footprint", p) # to drone frame
            except Exception:
                break

            loc = PoseWithCovarianceStamped(
                    p.header,
                    PoseWithCovariance(
                        p.pose,
                        cov
                    ))

            if (n.startswith('target')):
                #TODO(yoonyoungcho) : Green-Red Differentiation somehow?
                obs.append(Roomba(now, "/drone/base_footprint", Roomba.GREEN, loc))
            elif (n.startswith('obstacle')):
                obs.append(Roomba(now, "/drone/base_footprint", Roomba.OBSTACLE, loc))

        msg = RoombaSighting(
                header = Header(stamp=now, frame_id="/drone/base_footprint"),
                data = obs,
                fov_center = PointStamped(Header(stamp=now, frame_id="/drone/base_footprint"), Point(0,0,0)),
                fov_radius = R
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
