#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Vector3Stamped, PoseWithCovariance, TwistWithCovariance
from mavros_msgs.srv import SetMode, CommandTOL, StreamRate
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from numpy import sin, cos

from tf import TransformBroadcaster
import tf.transformations


class PixhawkConnector(object):
    def __init__(self):

	self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=0)
        self.odom_pub = rospy.Publisher('/odometry', Odometry, queue_size=10)

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
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

        # TODO: check this math for trig errors
        dt = (msg.header.stamp - self.last_pos.header.stamp).to_sec()
        if dt > 1:
            # It's been too long!
            dt = 0

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

        self.publish_pose(pose, msg.twist)

    def publish_pose(self, pose, twist, child_frame='fcu'):
        orientation = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                       pose.pose.orientation.w]
        position = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        print "Calculated pose: ", pose
        self.tfb.sendTransform(position, orientation, pose.header.stamp, child_frame,
                               pose.header.frame_id)

        odom = Odometry()
        odom.header = pose.header
        
        odom.pose = PoseWithCovariance(pose=pose.pose)
        odom.pose.covariance[0] = -1

        odom.twist = TwistWithCovariance(twist=twist.twist)
        odom.twist.covariance[0] = -1
        odom.child_frame_id = child_frame

        self.odom_pub.publish(odom)

    def setup_pixhawk(self, rate=160):
        rospy.wait_for_service('/mavros/set_mode')
        ok = self.set_mode(custom_mode='GUIDED')
        if not ok.mode_sent:
            rospy.logerr("Unable to set Pixhawk mode")
        ok = self.set_rate(message_rate=rate, on_off=True)
        rospy.loginfo("Setting stream rate to {}".format(rate))
        if not ok:
           rospy.logerr("Unable to set stream rate")

    def on_takeoff(self,msg):
        rospy.wait_for_service('/mavros/cmd/takeoff')
        res = self.takeoff(latitude=float("NaN"),longitude=float("NaN"),altitude=2)
        rospy.loginfo("Took Off:", res)

    def on_land(self,msg):
        rospy.wait_for_service('/mavros/cmd/land')
        res = self.land()
        rospy.loginfo("Landed:", res)

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
