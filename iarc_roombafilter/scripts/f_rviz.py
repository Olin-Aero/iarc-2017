#!/usr/bin/env python2

"""
RVIZ Interface with all the filtering stuff.
Should display what all the nodes produces.
Use for debugging purposes.
"""

import rospy
import tf

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from geometry_msgs.msg import Point, Quaternion
from iarc_main.msg import Roomba, RoombaSighting, RoombaList
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray

class RVIZInterface(object):
    def __init__(self):
        rospy.init_node("rviz_interface")

        self._drone = None
        self._tf = tf.TransformListener()

        self._gz = []
        self._obs = []
        self._est = []

        # subscribe ...
        self._gz_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gz_cb, queue_size=1)
        self._obs_sub = rospy.Subscriber("roomba_obs", RoombaSighting, self.obs_cb, queue_size=1)
        self._est_sub = rospy.Subscriber("roombas", RoombaList, self.est_cb, queue_size=1)

        # publish ...
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

    def gz_cb(self, msg):
        self._gz = [p for (n,p) in zip(msg.name, msg.pose) if (n.startswith('target') or n.startswith('obstacle'))]
        self._drone = msg.pose[msg.name.index("quadrotor")]

    def obs_cb(self, msg):
        h = Header(frame_id = msg.header.frame_id)
        obs = [r.visible_location.pose.pose for r in msg.data]
        try:
            self._obs = [self._tf.transformPose('map', PoseStamped(h, r)).pose for r in obs]
        except Exception as e:
            print e
            self._obs = []

    def est_cb(self, msg):
        h = Header(frame_id = msg.header.frame_id)
        est = [r.visible_location.pose.pose for r in msg.data]
        try:
            self._est = [self._tf.transformPose('map', PoseStamped(h, r)).pose for r in est]
        except Exception:
            self._est = []

    def publish(self):
        print dir(self._mks)
        self._mk_idx = 0
        if self._drone:
            self.add_marker(self._drone, [1,1,1, 1.0], 1.0)
            self.add_marker(self._drone, [1,1,1, 0.1], 3.0)
        for p in self._gz:
            self.add_marker(p, [1,0,0, 0.3], 1.0) # red = ground truth
        for p in self._obs:
            self.add_marker(p, [0,1,0, 0.2], 1.5) # green = observation
        for p in self._est:
            self.add_marker(p, [0,0,1, 0.5], 2.0) # blue = estimates
        for i in range(self._mk_idx, self._mk_max):
            self._mks.markers[i].color.a = 0.0

        self._pub.publish(self._mks)

    def add_marker(self, pose, color, scale):
        marker = self._mks.markers[self._mk_idx]
        marker.scale.x = 0.2 * scale
        marker.scale.y = 0.2 * scale
        marker.scale.z = 0.2 * scale
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
