#!/usr/bin/env python2

import numpy as np
import rospy
import tf
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point32, Polygon, PolygonStamped
from f_utils import observability

class ObservabilityViz(object):
    def __init__(self, camera_topic, out_topic='obs_ar'):
        rospy.init_node('obs_ar_rviz')
        self._tf = tf.TransformListener()
        self._pub = rospy.Publisher(out_topic, PolygonStamped, queue_size=10)
        self._sub = rospy.Subscriber(camera_topic, CameraInfo, self._ci_cb)
        self._msg = PolygonStamped()

        self._K = None
        self._w = None
        self._h = None
        self._frame = None

    def _ci_cb(self, msg):
        self._K = np.reshape(msg.K, (3,3))
        self._w = msg.width
        self._h = msg.height
        self._frame = msg.header.frame_id

    def _a2p(self, ar):
        return Polygon([Point32(*v) for v in ar])

    def _proc(self):
        try:
            if (self._K is not None) and self._w and self._h and self._frame:
                # looks a bit stupid ... TODO(yoonyoungcho): fix
                _, q = self._tf.lookupTransform(self._frame, 'map', rospy.Time(0))
                t, _ = self._tf.lookupTransform('map', self._frame, rospy.Time(0))
                q = [q[3], q[0], q[1], q[2]]
                print t
                #print 't', t, t2
                ar = observability(self._K, self._w, self._h, q, t, False)
                poly = self._a2p(ar)
                #print np.shape(ar) #
                self._msg.header.stamp = rospy.Time.now()
                self._msg.header.frame_id = 'map'
                self._msg.polygon = poly
                self._pub.publish(self._msg)
        except Exception as e:
            print 'Exception!!!!', e

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self._proc()
            rate.sleep()

        

if __name__ == "__main__":
    viz = ObservabilityViz('/ardrone/bottom/camera_info')
    viz.run()

