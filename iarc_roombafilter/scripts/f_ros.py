"""
ROS Interface to the Multi-Target Matching UKF Filter.
Configuration files should be modified under f_config.py

Takes RoombaObservation and filters them into pose.

TODO(yoonyoungcho) : expose dynamic_reconfigure
TODO(yoonyoungcho) : convert dt
TODO(yoonyoungcho) : verify fovcenter ~= Drone Position?
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from iarc_main.msg import RoombaSighting
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

        # create ROS interfaces
        self._sub = rospy.Subscriber(obs_topic, RoombaSighting, self.obs_Cb)
        self._pub = rospy.Publisher('', PoseWithCovarianceStamped, queue_size=10)
        self._tf = tf.TransformListener()
        self._mgr = UKFManager(dt, sigma)

    def obs_cb(self, msg):
        #self._tf.lookupTransform('map', 
        pos = self._tf.transformPoint(self, 'map', msg.fov_center)
        obs_ar = CircularObservation(
                pos.x,
                pos.y,
                3.0
                )
        pass

def main():
    rospy.init_node('roomba_filter')


    mgr = UKFManagerROS(sigma)

if __name__ == "__main__":
    main()
