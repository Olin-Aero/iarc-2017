import math
import rospkg

import rospy
import numpy as np
import sys
import os
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from iarc_arbiter.msg import RegisterBehavior
from std_msgs.msg import Float64, String
from tf import TransformListener
rospack = rospkg.RosPack()
iarc_sim_path = rospack.get_path('iarc_sim_2d')
sys.path.append(os.path.join(iarc_sim_path, 'src'))
import config as cfg

from Roomba import Roomba


class Drone:
    def __init__(self, roomba=None):
        self.normal_height = 5.0
        self.current_target = roomba
        self.actual_height = 0.0
        self.vel3d = Twist()

        self.frame_id = "base_link"
        self.max_vertical_vel = 1

        self.followPub = rospy.Publisher('/follow/cmd_pos', PoseStamped, queue_size=10)
        self.heightPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.tf = TransformListener()
        rospy.Subscriber("/drone/height", Float64, self.record_height)
        rospy.Subscriber("/cmd_vel", Twist, self.record_vel)

    def record_height(self, msg):
        self.actual_height = msg.data

    def record_vel(self, msg):
        self.vel3d = msg

    def follow_roomba(self, roomba=None, des_x=0, des_y=0, des_z=0):
        """
        Follow roomba X with desired position des_x, des_y
        :param roomba: an roomba object
        :param des_x: desired position x
        :param des_y: desired position y
        :param des_z: desired position z
        """
        if roomba is None:
            roomba = self.current_target
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = des_x
        pose_stamped.pose.position.y = des_y
        pose_stamped.pose.position.z = des_z
        pose_stamped.header.frame_id = roomba.frame_id

        self.followPub.publish(pose_stamped)

    def change_height(self, height):
        error = height - self.actual_height
        max_vel = np.sign(error) * self.max_vertical_vel
        if error < max_vel:
            max_vel = error

        self.vel3d.linear.z = max_vel

        self.heightPub.publish(self.vel3d)
        print "Current height:", self.actual_height
        return height - self.actual_height

    def push_roomba_button(self, button_hit=False, back_to_height=False):
        button_hit = button_hit
        back_to_height = back_to_height

        if self.distance_from_target() is None:
            print "No target to hit button"
            return True, True

        if not button_hit:
            # Hit the button
            if self.distance_from_target() < 0.17:
                self.change_height(cfg.ROOMBA_HEIGHT)

                # 0.1 is roomba's height
                if self.actual_height <= cfg.ROOMBA_HEIGHT + cfg.PAD_HEIGHT/2:
                    print "Button hit"
                    button_hit = True

        elif not back_to_height:
            # Return to normal height
            self.change_height(self.normal_height + 0.2)
            if self.actual_height >= self.normal_height:
                print "Back to normal height"
                back_to_height = True

        return button_hit, back_to_height

    def distance_from_target(self):
        if roomba is None:
            return None

        # Get the location of the target relative to the drone
        position, quaternion = self.tf.lookupTransform(self.frame_id, self.current_target.frame_id, rospy.Time(0))
        distance = math.sqrt(position[0] ** 2 + position[1] ** 2)
        return distance

    def test_follow_roomba(self):
        while self.distance_from_target() > 0.1:
            drone.follow_roomba()

    def test_change_height(self, height=0.0):
        height = height
        error = height - self.actual_height
        while abs(error) >= 0.1:
            error = self.change_height(height)

    def test_push_roomba(self):
        drone.test_change_height(height=2.0)
        button_hit, back_to_height = self.push_roomba_button()
        while not button_hit or not back_to_height:
            self.follow_roomba()
            button_hit, back_to_height = self.push_roomba_button(button_hit, back_to_height)


if __name__ == '__main__':
    rospy.init_node('Example')
    rospy.sleep(0.5)
    roomba = Roomba("target2")
    drone = Drone(roomba=roomba)

    # Register this behavior (FollowBehavior) to the arbiter
    rospy.Publisher('/arbiter/register', RegisterBehavior, latch=True, queue_size=10).publish(name='follow')

    # This command tell the arbiter that this behavior will take command from now on and abort other behaviors
    rospy.Publisher('/arbiter/activate_behavior', String, latch=True, queue_size=10).publish('follow')

    r = rospy.Rate(10)

    # while not rospy.is_shutdown():
    r.sleep()
    # drone.test_change_height()
    # drone.push_roomba_button()
    drone.test_push_roomba()
    # drone.test_change_height()
