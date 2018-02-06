#!/usr/bin/env python2

"""
RVIZ Interface with all the filtering stuff.
Should display what all the nodes produces.
Use for debugging purposes.
"""

import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from geometry_msgs.msg import Point, Quaternion
from iarc_main.msg import Roomba, RoombaSighting, RoombaList
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray

class RVIZInterface(object):
    def __init__(self):
        rospy.init_node("rviz_interface")

        self._gz = []
        self._obs = []
        self._est = []

        # subscribe ...
        self._gz_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gz_cb)
        self._obs_sub = rospy.Subscriber("roomba_obs", RoombaSighting, self.obs_cb)
        self._est_sub = rospy.Subscriber("roombas", RoombaList, self.est_cb)

        # publish ...
        self._pub = rospy.Publisher("roomba_markers", MarkerArray, queue_size=10)
        self._mks = MarkerArray()

    def gz_cb(self, msg):
        # list of poses w.r.t. map
        self._gz = [p for (n,p) in zip(msg.name, msg.pose) if (n.startswith('target') or n.startswith('obstacle'))]
    def obs_cb(self, msg):
        self._obs = [r.visible_location.pose.pose for r in msg.data]
    def est_cb(self, msg):
        self._est = [r.visible_location.pose.pose for r in msg.data]
    def publish(self):
        self._mks = MarkerArray()
        self._mk_idx = 0
        for p in self._gz:
            self.add_marker(p, [1,0,0, 0.3], 1.0) # red = ground truth
        for p in self._obs:
            self.add_marker(p, [0,1,0, 0.2], 1.5) # green = observation
        for p in self._est:
            self.add_marker(p, [0,0,1, 0.5], 2.0) # blue = estimates
        self._pub.publish(self._mks)
    def add_marker(self, pose, color, scale):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
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
