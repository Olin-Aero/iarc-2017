"""
ROS Interface to the Multi-Target Matching UKF Filter.
Configuration files should be modified under f_config.py

Takes RoombaObservation and filters them into pose.

TODO(yoonyoungcho) : expose dynamic_reconfigure
TODO(yoonyoungcho) : convert dt
TODO(yoonyoungcho) : verify fovcenter ~= Drone Position?
TODO(yoonyoungcho) : is model-based predictive simulation necessary?
TODO(yoonyoungcho) : resolve iterative prediction vs. sparse updates
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from iarc_main.msg import Roomba, RoombaSighting
import tf

# Filter-Related ...
from f_utils import *
import f_config as cfg
import numpy as np
import cv2

from f_manager import UKFManager

class UKFManagerROS(object):
    def __init__(self, dt, sigma):

        # unroll parameters ...
        self._max_targets = rospy.get_param('~max_targets', default=30)
        self._sigma = rospy.get_param('~sigma', default=cfg.SIGMAS)
        self._p_keep = rospy.get_param('~p_keep', default=cfg.P_KEEP)
        self._p_match = rospy.get_param('~p_match', default=cfg.P_MATCH)
        self._p_clear = rospy.get_param('~p_clear', default=cfg.P_CLEAR)
        self._obs_topic = rospy.get_param('~obs_topic', default='roomba_obs')
        self._rate = rospy.get_param('~rate', default=100)

        # create ROS interfaces
        self._sub = rospy.Subscriber(obs_topic, RoombaSighting, self.obs_cb)
        self._pub = rospy.Publisher('', RoombaList, queue_size=10)
        self._tf = tf.TransformListener()
        self._mgr = UKFManager(dt, sigma)
        self._t = rospy.Time.now()

    def obs_cb(self, msg):
        """
        Perform filtering according to observation.
        """
        pos = self._tf.transformPoint('map', msg.fov_center)
        obs_ar = CircularObservation(
                pos.x,
                pos.y,
                msg.fov_radius
                )

        # format observation data ...
        obs = []
        for r in msg.data:
            p = self._tf.transformPose('map', r.visible_location.pose.pose)
            p, q = p.position, p.orientation
            h = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
            pose = [p.position.x, p.position.y, h, np.nan, np.nan]
            # TODO(yoonyoungcho) : is current code handling NaN well?

            # roomba type handling
            r_type = cfg.T_NULL
            r_col = cfg.C_NULL
            if r.type != Roomba.UNKNOWN:
                if r.type == Roomba.OBSTACLE:
                    r_type = cfg.T_OBST
                else:
                    r_type = cfg.T_TARG
                    r_col = (cfg.C_RED if r.type == Roomba.RED else cfg.C_GREEN)
            
            o = ObservationParticle(
                    pose=pose,
                    t=r_type,
                    c=r_col
                    )
            obs.append(o)

        # TODO(yoonyoungcho) does filterpy support retrodictive updates?
        # TODO(yoonyoungcho) t = t0+dt or t1-dt?
        t = msg.header.stamp.to_sec()
        self._mgr.step(obs, t, t-self._t, obs_ar)

    def publish(self):
        pass

    def run(self):
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            self._t = rospy.Time.now().to_sec()
            self.publish()
            rate.sleep()


def main():
    rospy.init_node('roomba_filter')
    mgr = UKFManagerROS(sigma)

if __name__ == "__main__":
    main()
