#!/usr/bin/env python2
"""
Prediction Engine
Written by Eric Miller

This file contains the object-form of the prediction engine. Ultimately, all code in heatmap.py
should move here.
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from iarc_main.msg import Roomba
`


class PredictionEngine(object):
    def __init__(self, tf_listener=None):
        # if tf_listener is None:
        #     tf_listener = tf.TransformListener()
        #
        # self.tf = tf_listener

        pass

    def predict_future_position(self, roomba, time):
        """
        Predicts the future position of a Roomba at time t
        :param (Roomba) roomba: The Roomba object
        :param (rospy.Time) time: The time in the future when we are interested in knowing the position
        :return: Estimated position, with uncertainty
        :rtype: PoseWithCovarianceStamped
        """

        pose = roomba.visible_location

        return fiveSecSim(roomba.last_turn, pose, time)

    def ros_node(self):
        """
        Sets up service handlers for this being a fully-functional ROS node
        :return:
        """
        rospy.logerr('Service handlers unimplemented')
        rospy.spin()


if __name__ == '__main__':
    PredictionEngine().ros_node()
