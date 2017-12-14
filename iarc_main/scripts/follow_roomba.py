#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from iarc_arbiter.msg import RegisterBehavior
from std_msgs.msg import String
from iarc_main.msg import Roomba


class RoombaFollower(object):
    def __init__(self):
        self.pos_pub = rospy.Publisher('/follow/cmd_pos', PoseStamped, queue_size=10)
        self.stop_pub = rospy.Publisher('/follow/cmd_vel', Twist, queue_size=10)

        # Register this behavior (FollowBehavior) to the arbiter
        rospy.Publisher('/arbiter/register', RegisterBehavior, latch=True, queue_size=10).publish(name='follow')

        self.target_sub = rospy.Subscriber('/target', Roomba, self.on_target)

        # A target with '' as a frame_id represents not having an active target
        self.latest_target = Roomba(frame_id='')

    def on_target(self, msg):
        """
        :param (Roomba) msg: The current target of the robot
        :return: None
        """
        self.latest_target = msg

    def follow_roomba(self, des_x=0, des_y=0, des_z=2.0):
        """
        Follow the latest received Roomba
        :param des_x: desired position x
        :param des_y: desired position y
        :param des_z: desired position z
        """
        if self.latest_target.frame_id == '':
            rospy.logerr_throttle(5, "No target roomba set, commanding 0 velocity.")
            self.stop_pub.publish(Twist())
            return

        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = des_x
        pose_stamped.pose.position.y = des_y
        pose_stamped.pose.position.z = des_z
        pose_stamped.header.frame_id = self.latest_target.frame_id
        pose_stamped.header.stamp = rospy.Time(0)

        self.pos_pub.publish(pose_stamped)

    def run(self):
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            r.sleep()
            self.follow_roomba()

    def test(self):
        # Create a target object just for testing
        fake_target = Roomba(frame_id="target1")

        # This command tell the arbiter that this behavior will take command from now on and abort other behaviors
        rospy.Publisher('/arbiter/activate_behavior', String, latch=True, queue_size=10).publish('follow')

        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            r.sleep()
            self.on_target(fake_target)
            self.follow_roomba()


if __name__ == '__main__':
    # Init FollowBehavior node
    rospy.init_node('FollowBehavior')

    RoombaFollower().run()
