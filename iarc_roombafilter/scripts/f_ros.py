#!/usr/bin/env python2

"""
ROS Interface to the Multi-Target Matching UKF Filter.
Configuration files should be modified under f_config.py

Takes RoombaObservation and filters them into pose.

TODO(yoonyoungcho) : expose dynamic_reconfigure
TODO(yoonyoungcho) : figure out world-time synchronization
TODO(yoonyoungcho) : dynamic dt
TODO(yoonyoungcho) : situational covariance updates (unobservable, should-be-observable, add-noise-event, reversal-event, ...)
TODO(yoonyoungcho) : is model-based predictive simulation necessary?
"""

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, Quaternion, Transform, TransformStamped
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from iarc_main.msg import Roomba, RoombaList
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

import f_config as cfg
from f_manager import UKFManager
# Filter-Related ...
from f_utils import *
from f_model import TargetRoombaModel, ObstacleRoombaModel


class UKFManagerROS(object):
    def __init__(self, dt):

        # unroll parameters ...
        self._max_targets = rospy.get_param('~max_targets', default=30)
        self._sigma = np.copy(rospy.get_param('~sigma', default=list(cfg.SIGMAS)))
        Particle.set_sigmas(np.copy(self._sigma))
        self._p_keep = rospy.get_param('~p_keep', default=cfg.P_KEEP)
        self._p_match = rospy.get_param('~p_match', default=cfg.P_MATCH)
        self._p_clear = rospy.get_param('~p_clear', default=cfg.P_CLEAR)
        self._cam_topic = rospy.get_param('~cam_topic', default='/ardrone/bottom/camera_info')
        self._obs_topic = rospy.get_param('~obs_topic', default='visible_roombas')
        self._out_topic = rospy.get_param('~out_topic', default='seen_roombas')
        self._sim2d = rospy.get_param('~sim2d', default=False)
        self._rate = rospy.get_param('~rate', default=50)  # 50hz

        # create ROS interfaces
        self._tf = tf.TransformListener(True, cache_time=rospy.Duration(5))
        self.tf_pub = tf.TransformBroadcaster()
        self._cam_sub = rospy.Subscriber(self._cam_topic, CameraInfo, self.cam_cb, queue_size=1)
        self._sub = rospy.Subscriber(self._obs_topic, RoombaList, self.obs_cb, queue_size=1)
        self._pub = rospy.Publisher(self._out_topic, RoombaList, queue_size=10)
        self._mgr = UKFManager(dt, self._sigma)
        self._t = rospy.Time.now()

        self._obs = None
        self._cam_frame = ''

    def cam_cb(self, msg):
        """ Get camera info once, and quit """
        self._K = np.reshape(msg.K, (3, 3))
        self._w = msg.width
        self._h = msg.height
        self._cam_frame = msg.header.frame_id

        if self._cam_sub:
            self._cam_sub.unregister()
            self._cam_sub = None

    def obs_cb(self, msg):
        """
        Save observation for processing ...
        """
        self._obs = msg

    def obs_proc(self):
        """
        Perform filtering according to observation.
        Decoupled from obs_cb to only process latest observation.
        """
        msg = self._obs
        self._obs = None
        if msg == None:
            return

        if self._sim2d:
            try:
                pos, _ = self._tf.lookupTransform('map', 'base_link', rospy.Time(0))
                obs_ar = ConicObservation(
                        pos[0],
                        pos[1],
                        pos[2],
                        aov=np.pi*2/3
                        )
            except tf.Exception as e:
                rospy.loginfo_throttle(1.0, 'Sim2D TF Failed : {}'.format(e))
                return
        else:
            try:
                _, q = self._tf.lookupTransform(self._cam_frame, 'map', rospy.Time(0))
                t, _ = self._tf.lookupTransform('map', self._cam_frame, rospy.Time(0))
                q = [q[3], q[0], q[1], q[2]]  # reorder xyzw-> wxyz
                ar = observability(self._K, self._w, self._h, q, t, False)
                obs_ar = PolygonObservation(ar)
            except tf.Exception as e:
                rospy.loginfo_throttle(5, 'Falling back to conic observation {}'.format(e))
                try:
                    t, _ = self._tf.lookupTransform('base_link', 'map', msg.header.stamp)
                    obs_ar = ConicObservation(t[0], t[1], t[2], 2)
                except tf.Exception as e:
                    print "That didn't work...", e
                    return

        # format observation data ...
        obs = []
        for r in msg.data:

            # print r.visible_location.header.frame_id
            # print r.visible_location.pose.pose.position.x

            p = PoseStamped(r.visible_location.header, r.visible_location.pose.pose)
            p = self._tf.transformPose('map', p).pose
            p, q = p.position, p.orientation
            h = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

            pose = np.asarray([p.x, p.y, h])
            # note : no velocity information is available.

            # roomba type handling
            # todo : use enums? or leave independent
            r_type = cfg.T_NULL
            if r.type == Roomba.OBSTACLE:
                r_type = cfg.T_OBST
            else:
                r_type = cfg.T_RED if r.type == Roomba.RED else cfg.T_GREEN

            o = ObservationParticle(
                pose=pose,
                t=r_type,
            )
            obs.append(o)

        # t = msg.header.stamp.to_sec()
        t = rospy.Time.now().to_sec()
        dt = t - self._t
        if dt > 0:
            self._mgr.step(obs, t, dt, obs_ar)
            self._t = t

    def publish(self):
        now = rospy.Time.now()
        cov = np.eye(6) * 1e-6  # TODO : do real covs
        cov = cov.flatten()
        msgs = []

        # try:
        #    print self._mgr.estimates()[0]._pose
        # except Exception as e:
        #    pass

        for k, e in self._mgr.estimates().iteritems():
            (x, y, t, v, w) = e._pose
            # print x, v
            s, c = np.sin(t / 2.0), np.cos(t / 2.0)
            p = Point(x, y, 0)
            q = Quaternion(0, 0, s, c)
            loc = PoseWithCovarianceStamped(
                Header(stamp=now, frame_id="map"),
                PoseWithCovariance(
                    Pose(p, q),
                    cov
                ))
            # todo : fill in type information here
            roomba_frame = "filtered/{}".format(k)

            msg = Roomba(last_seen=now, frame_id=roomba_frame, type=Roomba.RED, visible_location=loc)

            self.tf_pub.sendTransformMessage(
                TransformStamped(
                    header=msg.visible_location.header,
                    child_frame_id=msg.frame_id,
                    transform=Transform(
                        translation=msg.visible_location.pose.pose.position,
                        rotation=msg.visible_location.pose.pose.orientation
                    )
                )
            )

            msgs.append(msg)
        msg = RoombaList(Header(stamp=now, frame_id="map"), msgs)
        # print len(msgs)
        self._pub.publish(msg)

    def run(self):
        rate = rospy.Rate(self._rate)
        self._t = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            # self._t = rospy.Time.now().to_sec()
            self.obs_proc()
            self.publish()
            rate.sleep()


def main():
    rospy.init_node('roomba_filter')

    # configure models by params
    TargetRoombaModel.configure({
        # speed configurations
        'v'    : 0.33,
        'w'    : 1.375,
        # intervals
        't_n'  : cfg.INT_NOISE, # noise interval
        't_r'  : cfg.INT_REVERSE, # reversal interval
        # durations
        'd_180': cfg.T_180,
        'd_45' : cfg.T_45,
        'd_n'  : cfg.T_NOISE
        })
    ObstacleRoombaModel.configure({
        'v'    : 0.33,
        'w'    : 0.066
        })

    mgr = UKFManagerROS(dt=1e-2)
    mgr.run()


if __name__ == "__main__":
    main()
