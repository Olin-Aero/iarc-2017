#!/usr/bin/env python2
import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovariance, TwistWithCovariance
from mavros_msgs.srv import SetMode, CommandTOL, StreamRate
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Header
from tf import TransformBroadcaster


class PixhawkConnector(object):
    def __init__(self):

        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=0)
        self.odom_pub = rospy.Publisher('/odometry', Odometry, queue_size=10)

        self.tfb = TransformBroadcaster()

        rospy.sleep(0.5)

        startpose = PoseStamped()
        startpose.header.stamp = rospy.Time.now()
        startpose.header.frame_id = 'odom'
        startpose.pose.orientation.w = 1
        self.publish_pose(startpose, TwistWithCovariance())

        rospy.loginfo("Published initial tf")

        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_rate = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
        self.land = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        rospy.loginfo('Services connected')

        self.vel = Twist()
        self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.on_vel)

        self.takeoff_sub = rospy.Subscriber('/takeoff', Empty, self.on_takeoff)
        self.land_sub = rospy.Subscriber('/land', Empty, self.on_land)

        self.last_pose = PoseStamped()
        self.position_sub = rospy.Subscriber('/mavros/global_position/local', Odometry, self.on_pose)

    def on_pose(self, msg):
        """
        :param Odometry msg:
        """
        if (msg.header.stamp - rospy.Time.now()).to_sec() > 0:
            rospy.logwarn_throttle(1.0, "Got an odometry message from the future, correcting.")
            msg.header.stamp = rospy.Time.now()

        vel = msg.twist.twist.linear
        alt = msg.pose.pose.position.z
        orientation = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

        dt = (msg.header.stamp - self.last_pose.header.stamp).to_sec()
        if dt > 1 or dt < 0:
            # It's been too long!
            dt = 0
            rospy.logwarn("Erroneously large dt detected, skipping.")
        if vel.x > 1000 or vel.y > 1000:
            vel.x = vel.y = 0
            rospy.logwarn("Erroneously large velocities detected, skipping.")

        # The velocity retrieved is (we think) in North - East - Down, while ROS normally uses East - North - Up
        self.last_pose.pose.position.x += dt * vel.y
        self.last_pose.pose.position.y += dt * vel.x
        self.last_pose.pose.position.z = alt

        self.last_pose.pose.orientation = orientation

        self.last_pose.header.stamp = msg.header.stamp

        pose = PoseStamped()

        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'odom'

        pose.pose.orientation = orientation
        pose.pose.position = self.last_pose.pose.position

        self.publish_pose(pose, msg.twist)

    def publish_pose(self, pose, twist, child_frame='base_link'):
        orientation = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                       pose.pose.orientation.w]
        position = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        rospy.loginfo_throttle(1.0, "Calculated pose: {}".format(pose))
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

    def on_takeoff(self, msg):
        rospy.wait_for_service('/mavros/cmd/takeoff')
        res = self.takeoff(latitude=float("NaN"), longitude=float("NaN"), altitude=2)
        rospy.loginfo("Took Off")

    def on_land(self, msg):
        rospy.wait_for_service('/mavros/cmd/land')
        res = self.land()
        rospy.loginfo("Landed")

    def on_vel(self, msg):
        """
        :param Twist msg:
        :return: None
        """
        q = self.last_pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # TODO: check the signs on these math equations
        self.vel.linear.z = msg.linear.z
        self.vel.linear.x = msg.linear.x * np.cos(yaw) + msg.linear.y * np.sin(yaw)
        self.vel.linear.y = msg.linear.y * np.cos(yaw) - msg.linear.x * np.sin(yaw)

        rospy.loginfo_throttle(0.5, "Vel correction: \tth=({}) \tin={} \tout={}".format(
            yaw, (msg.linear.x, msg.linear.y), (self.vel.linear.x, self.vel.linear.y)))

        self.vel.angular = msg.angular
        try:
            self.vel_pub.publish(self.vel)
        except:
            rospy.logerr("Unable to publish velocity")

    def run(self):
        self.setup_pixhawk()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('pixhawk_connector')
    PixhawkConnector().run()
