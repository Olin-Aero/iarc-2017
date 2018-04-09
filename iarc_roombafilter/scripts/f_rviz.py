#!/usr/bin/env python2

"""
RVIZ Interface with all the filtering stuff.
Should display what all the nodes produces.
Use for debugging purposes.
"""

import numpy as np
import rospy
import tf

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point32, Polygon, PolygonStamped
from iarc_main.msg import Roomba, RoombaList
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray

from f_utils import observability

class RVIZInterface(object):
    def __init__(self):
        rospy.init_node("rviz_interface")

        # Flag : show ground truth from gazebo
        self._show_gz = rospy.get_param('~show_gz', default=False)
        # Flag : show input observations 
        self._show_obs = rospy.get_param('~show_obs', default=True)
        # Flag : show output filtered estimates
        self._show_est = rospy.get_param('~show_est', default=True)
        # Flag : 2D Sim, fake observation area
        self._sim2d = rospy.get_param('~sim2d', default=True)

        self._drone = None
        self._tf = tf.TransformListener()

        self._gz = []
        self._obs = []
        self._est = []

        self._K = None
        self._w = None
        self._h = None
        self._cam_frame = None

        if self._show_gz:
            # show ground truth from gazebo
            self._gz_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gz_cb, queue_size=1)

        if self._show_obs:
            # observation particles
            self._obs_sub = rospy.Subscriber("visible_roombas", RoombaList, self.obs_cb, queue_size=1)
            # observation area
            self._cam_sub = rospy.Subscriber('/ardrone/bottom/camera_info', CameraInfo, self.cam_cb, queue_size=1)
            self._ar_pub = rospy.Publisher("obs_ar", PolygonStamped, queue_size=10)
            self._ar_msg = PolygonStamped()

        if self._show_est:
            # show estimates
            self._est_sub = rospy.Subscriber("seen_roombas", RoombaList, self.est_cb, queue_size=1)

        # publish points ...
        self._pub = rospy.Publisher("roomba_markers", MarkerArray, queue_size=10)
        self._mks = MarkerArray()
        self._mk_max = 50 # 14 * 3 + 2 + leeway
        for i in range(self._mk_max):
            m = Marker()
            m.header.frame_id="map"
            m.type = m.SPHERE
            m.action = m.ADD
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color.r = m.color.g = m.color.b = 1.0
            m.color.a = 0.0
            self._mks.markers.append(m)

    def _a2p(self, ar):
        """ Area --> Polygon, helper for showing observability. """
        return Polygon([Point32(*v) for v in ar])

    def cam_cb(self, msg):
        """ Get camera info once, and quit """
        self._K = np.reshape(msg.K, (3,3))
        self._w = msg.width
        self._h = msg.height
        self._cam_frame = msg.header.frame_id

        if self._cam_sub:
            self._cam_sub.unregister()
            self._cam_sub = None

    def gz_cb(self, msg):
        """ Ground truth information callback from Gazebo """
        self._gz = [p for (n,p) in zip(msg.name, msg.pose) if (n.startswith('target') or n.startswith('obstacle'))]
        self._drone = msg.pose[msg.name.index("quadrotor")]

    def obs_cb(self, msg):
        """ Observation callback from perception source """
        h = Header(frame_id = msg.header.frame_id)
        obs = [r.visible_location.pose.pose for r in msg.data]
        try:
            self._obs = [self._tf.transformPose('map', PoseStamped(h, r)).pose for r in obs]
        except Exception as e:
            rospy.logerr_throttle(1.0, 'TF Exception at obs_cb() in f_rviz.py : {}/{}'.format(e, msg))
            self._obs = []

    def est_cb(self, msg):
        """ Estimation callback from f_ros.py/UKFManagerROS """
        h = Header(frame_id = msg.header.frame_id)
        est = [r.visible_location.pose.pose for r in msg.data]
        try:
            self._est = [self._tf.transformPose('map', PoseStamped(h, r)).pose for r in est]
        except Exception:
            self._est = []

    def _publish_area(self):
        try:
            if self._sim2d:
                t, _ = self._tf.lookupTransform('map', 'base_link', rospy.Time(0))
                x,y,z = t
                r = z * np.tan(np.pi/3)
                th = np.linspace(-np.pi, np.pi)
                ar_x = x + r*np.cos(th)
                ar_y = y + r*np.sin(th)
                ar = np.stack([ar_x, ar_y, 0*ar_y], axis=-1)
                poly = self._a2p(ar)
                self._ar_msg.header.stamp = rospy.Time.now()
                self._ar_msg.header.frame_id = 'map'
                self._ar_msg.polygon = poly
                self._ar_pub.publish(self._ar_msg)
            elif self._cam_frame:
                # looks a bit stupid ... TODO(yoonyoungcho): fix
                _, q = self._tf.lookupTransform(self._cam_frame, 'map', rospy.Time(0))
                t, _ = self._tf.lookupTransform('map', self._cam_frame, rospy.Time(0))
                q = [q[3], q[0], q[1], q[2]]
                ar = observability(self._K, self._w, self._h, q, t, False)
                poly = self._a2p(ar)
                self._ar_msg.header.stamp = rospy.Time.now()
                self._ar_msg.header.frame_id = 'map'
                self._ar_msg.polygon = poly
                self._ar_pub.publish(self._ar_msg)

        except Exception as e:
            rospy.logerr_throttle(1.0, 'Exception at _publish_area() in f_rviz.py : {}'.format(e))


    def publish(self):
        if self._show_obs:
            self._publish_area()

        self._mk_idx = 0
        #TODO(yoonyoungcho) : get drone pose information from tf, when gazebo source is unavailable
        if self._drone:
            self.add_marker(self._drone, [1,1,1, 1.0], 0.2)
            #self.add_marker(self._drone, [1,1,1, 0.1], 2*3.0) #+-3m
        if self._show_gz:
            for p in self._gz:
                self.add_marker(p, [1,0,0, 0.3], 0.2) # red = ground truth
        if self._show_obs:
            for p in self._obs:
                self.add_marker(p, [0,1,0, 0.2], 0.3) # green = observation
        if self._show_est:
            for p in self._est:
                self.add_marker(p, [0,0,1, 0.5], 0.4) # blue = estimates

        # hide all other markers
        # to prevent rviz from persisting them
        for i in range(self._mk_idx, self._mk_max):
            self._mks.markers[i].color.a = 0.0

        self._pub.publish(self._mks)

    def add_marker(self, pose, color, scale):
        """
        Set marker pose/color/scale.
        Note that marker index is automatically assigned.
        """
        marker = self._mks.markers[self._mk_idx]
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        if len(color) >= 4:
            marker.color.a = color[3]
        else:
            marker.color.a = 1.0
        marker.pose = pose
        marker.id = self._mk_idx
        self._mk_idx += 1
        self._mks.markers.append(marker)
            
    def run(self):
        """ Run the node """
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            #print 'gz', self._gz
            #print 'obs', self._obs
            #print 'est', self._est
            self.publish()
            rate.sleep()

def main():
    ri = RVIZInterface()
    ri.run()

if __name__ == "__main__":
    main()
