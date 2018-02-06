#!/usr/bin/env python2

"""
Gazebo Ground Truth Interface.
Intended as placeholder until actual roomba sighting code works.
"""

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from iarc_main.msg import Roomba, RoombaSighting
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, PointStamped

R = 10.0

def is_visible(src, dst):
    # dst pose is visible from src pose
    p0 = src.position
    p1 = dst.position
    r2 = (p0.x - p1.x)**2 + (p0.y - p1.y)**2
    return (r2 < R**2) # within +-3m radius

class GazeboInterface(object):
    def __init__(self):
        rospy.init_node("gz_interface")
        self._sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gz_cb)
        self._pub = rospy.Publisher('roomba_obs', RoombaSighting, queue_size=10)
    def gz_cb(self, msg):
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

            loc = PoseWithCovarianceStamped(
                    Header(stamp=now, frame_id="map"),
                    PoseWithCovariance(
                        p,
                        cov
                    ))

            if (n.startswith('target')):
                #TODO(yoonyoungcho) : Green-Red Differentiation somehow?
                obs.append(Roomba(now, "map", Roomba.GREEN, loc))
            elif (n.startswith('obstacle')):
                obs.append(Roomba(now, "map", Roomba.OBSTACLE, loc))


        msg = RoombaSighting(
                header = Header(stamp=now, frame_id="map"),
                data = obs,
                fov_center = PointStamped(Header(stamp=now, frame_id="map"), p0.position),
                fov_radius = R
                )
        self._pub.publish(msg)

    def run(self):
        rospy.spin()

def main():
    gi = GazeboInterface()
    gi.run()

if __name__ == "__main__":
    main()
