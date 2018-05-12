#!/usr/bin/env python2
import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovariance, TwistWithCovariance
from mavros_msgs.srv import SetMode, CommandTOL, StreamRate
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf import TransformBroadcaster


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

        self.last_pose = PoseStamped()
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

        dt = (msg.header.stamp - self.last_pose.header.stamp).to_sec()
        if dt > 1:
            # It's been too long!
            dt = 0

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
        # ok = self.set_mode(custom_mode='GUIDED')
        # if not ok.mode_sent:
        #     rospy.logerr("Unable to set Pixhawk mode")
        ok = self.set_rate(message_rate=rate, on_off=True)
        rospy.loginfo("Setting stream rate to {}".format(rate))
        if not ok:
            rospy.logerr("Unable to set stream rate")

    def on_takeoff(self):
        rospy.wait_for_service('/mavros/cmd/takeoff')
        res = self.takeoff()
        rospy.loginfo("Took Off:", res)

    def on_land(self):
        rospy.wait_for_service('/mavros/cmd/land')
        res = self.land()
        rospy.loginfo("Landed:", res)

    def on_vel(self, msg):
        """
        :param Twist msg:
        :return: None
        """
        q = self.last_pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # TODO: check the signs on these math equations
        self.vel.linear.z = msg.linear.z
        self.vel.linear.x = msg.linear.x*np.cos(yaw) + msg.linear.y*np.sin(yaw)
        self.vel.linear.y = msg.linear.y*np.cos(yaw) - msg.linear.x*np.sin(yaw)

        self.vel.angular = msg.angular

        self.vel_pub.publish(self.vel)

    def run(self):
        self.setup_pixhawk()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('pixhawk_connector')
    PixhawkConnector().run()
