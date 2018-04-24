#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Vector3Stamped
from mavros_msgs.srv import SetMode, CommandTOL, StreamRate
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from numpy import sin, cos

from tf import TransformBroadcaster
import tf.transformations


class PixhawkConnector(object):
    def __init__(self):
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=0)

        rospy.wait_for_service('/mavros/set_mode')
        rospy.sleep(0.2)
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_rate = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
        self.land = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        rospy.loginfo('Services connected')

        self.tfb = TransformBroadcaster()

        self.vel = Twist()
        self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.on_vel)

        self.takeoff_sub = rospy.Subscriber('/takeoff', Empty, self.on_takeoff)
        self.land_sub = rospy.Subscriber('/land', Empty, self.on_land)

        self.last_pos = Vector3Stamped()
        self.position_sub = rospy.Subscriber('/mavros/global_position/local', Odometry, self.on_pose)

    def on_pose(self, msg):
        """
        :param Odometry msg:
        """
        vel = msg.twist.twist.linear
        alt = msg.pose.pose.position.z
        orientation = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(orientation)

        # TODO: check this math for trig errors
        dt = (msg.header.stamp - self.last_pos.header.stamp).to_sec()

        # The velocity retrieved is (we think) in North - East - Down, while ROS normally uses East - North - Up
        self.last_pos.vector.x += dt * vel.y
        self.last_pos.vector.y += dt * vel.x
        self.last_pos.vector.z = alt

        self.last_pos.header.stamp = msg.header.stamp

        pose = PoseStamped()

        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'odom'

        pose.pose.orientation = orientation
        pose.pose.position = self.last_pos.vector

        self.publish_pose(pose)

    def publish_pose(self, pose):
        print "Calculated pose: ", pose
        self.tfb.sendTransform(pose.pose.position, pose.pose.orientation, pose.header.stamp, 'fcu',
                               pose.header.frame_id)
        # TODO: Publish Odometry message as well

    def setup_pixhawk(self, rate=160):
        # ok = self.set_mode(custom_mode='GUIDED')
        # if not ok.mode_sent:
        #     rospy.logerr("Unable to set Pixhawk mode")
        ok = self.set_rate(message_rate=rate)
        rospy.loginfo("Setting stream rate to {}".format(rate))
        if not ok:
            rospy.logerr("Unable to set stream rate")

    def on_takeoff(self):
        res = self.takeoff()
        rospy.loginfo("Took Off:", res)
        pass

    def on_land(self):
        res = self.land()
        rospy.loginfo("Landed:", res)
        pass

    def on_vel(self, msg):
        """
        :param Twist msg:
        :return: None
        """
        self.vel = msg
        self.vel_pub.publish(self.vel)

    def run(self):
        self.setup_pixhawk()
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pixhawk_connector')
    PixhawkConnector().run()
