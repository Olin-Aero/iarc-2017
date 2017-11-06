import math
import rospkg

import rospy
import numpy as np
import sys
import os
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from iarc_arbiter.msg import RegisterBehavior
from std_msgs.msg import Float64, String
import tf

rospack = rospkg.RosPack()
iarc_sim_path = rospack.get_path('iarc_sim_2d')
sys.path.append(os.path.join(iarc_sim_path, 'src'))
import config as cfg

from Roomba import Roomba


class Drone:
    def __init__(self, target=None):
        self.should_hit_button = False
        self.should_land_front = False
        self.actual_height = 0.0
        self.vel3d = Twist()
        self.MAX_VERTICAL_VEL = 2.0
        self.MIN_VERTICAL_VEL = 0.8
        self.NORMAL_HEIGHT = 2.5

        self.current_target = target
        self.prev_target_facing_angle = None  # in radian

        self.FRAME_ID = "base_link"
        self.tf = tf.TransformListener()
        self.followPub = rospy.Publisher('/follow/cmd_pos', PoseStamped, queue_size=10)
        self.heightPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/drone/height", Float64, self.record_height)
        rospy.Subscriber("/cmd_vel", Twist, self.record_vel)

    def record_height(self, msg):
        # print self.actual_height
        self.actual_height = msg.data

    def record_vel(self, msg):
        self.vel3d = msg

    def follow_roomba(self, roomba=None, des_x=0.0, des_y=0.0, des_z=0.0):
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

    def follow_roomba_global(self, des_x=0.0, des_y=0.0, des_z=0.0):
        """
        Follow roomba X with desired position des_x, des_y
        :param roomba: an roomba object
        :param des_x: desired position x
        :param des_y: desired position y
        :param des_z: desired position z
        """
        if self.current_target is None:
            return
        position = self.position_of_roomba()
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = position[0] + des_x
        pose_stamped.pose.position.y = position[1] + des_y
        pose_stamped.pose.position.z = des_z
        pose_stamped.header.frame_id = "map"

        self.followPub.publish(pose_stamped)

    def change_height(self, height):
        """
        Change drone's height to height
        :param height: Desired height
        :return: Distance to height
        """
        error = height - self.actual_height
        min_vel = np.sign(error) * self.MIN_VERTICAL_VEL
        max_vel = np.sign(error) * self.MAX_VERTICAL_VEL

        if error < min_vel:
            vel = min_vel
        elif error > max_vel:
            vel = max_vel
        else:
            vel = error

        self.vel3d.linear.z = vel

        self.heightPub.publish(self.vel3d)

        return height - self.actual_height

    def stand_still(self):
        self.vel3d = Twist()
        self.heightPub.publish(self.vel3d)

    def push_button(self):
        if self.distance_from_target() is None:
            print "No target to hit button"
            self.should_hit_button = False

        if self.should_hit_button:
            # Hit the button
            self.change_height(cfg.ROOMBA_HEIGHT - 0.1)
            if self.distance_from_target() < 0.1:

                # 0.1 is roomba's height
                if self.actual_height <= cfg.ROOMBA_HEIGHT:
                    print "Button hit"
                    self.should_hit_button = False
                    self.change_height(self.NORMAL_HEIGHT)

    def land_front(self):
        if self.distance_from_target() is None:
            print "No target to stand in front"
            self.should_land_front = False

        if self.should_land_front:
            # Land in front
            if self.vel3d.linear.x != 0 or self.vel3d.linear.y != 0:
                self.follow_roomba(des_x=0.6)
            self.change_height(0)

            if self.actual_height <= 0.1:
                if self.distance_from_target() <= 0.35:
                    print "Taking off"
                    self.should_land_front = False
                    self.follow_roomba(des_x=0)
                    self.change_height(self.NORMAL_HEIGHT)
                elif self.distance_from_target() <= 0.8:
                    print "Landed in front and waiting for roomba"
                    self.stand_still()

    def distance_from_target(self):
        """
        Get the distance from the target relative to drone
        :return: distance
        """
        if roomba is None:
            return None

        position, quaternion = self.tf.lookupTransform(self.FRAME_ID, self.current_target.frame_id, rospy.Time(0))
        distance = math.sqrt(position[0] ** 2 + position[1] ** 2)
        return distance

    def position_of_roomba(self):
        """
        Get the distance from the target relative to drone
        :return: distance
        """
        if roomba is None:
            return None

        position, quaternion = self.tf.lookupTransform("map", self.current_target.frame_id, rospy.Time(0))
        return position

    def target_facing_angle(self):
        """
        :return: The angle the target is facing from PI to -PI & True if target is rotating
        """
        if roomba is None:
            return None

        position, quaternion = self.tf.lookupTransform("map", self.current_target.frame_id, rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(quaternion)
        is_rotating = euler[-1] != self.prev_target_facing_angle
        # print euler[-1] / np.pi * 180
        self.prev_target_facing_angle = euler[-1]
        return euler[-1], is_rotating

    def check_back_to_height(self):
        if self.actual_height <= self.NORMAL_HEIGHT:
            return False
        # print "Back to normal height"
        return True

    def check_push_button(self):
        """
        Check if the target is facing to the right direction
        :return: True: need to push button, False: otherwise
        """
        angle, is_rotating = self.target_facing_angle()
        if angle is None or is_rotating:
            return False
        pi = np.pi
        return -pi / 3 <= angle <= pi / 3 or angle >= 2 * pi / 3 or angle <= -2 * pi / 3

    def check_land_front(self):
        """
        Check if the target is facing downward from -pi/4 to -3 pi/4(
        :return: True: need to land in front, False: otherwise
        """
        angle, is_rotating = self.target_facing_angle()
        if angle is None or is_rotating:
            return False
        pi = np.pi
        return -2 * pi / 3 < angle < -pi / 3

    """
    These are methods for testing purposes
    """

    def test_follow_roomba(self):
        self.test_change_height(3)
        r = rospy.Rate(10)
        while True:
            # pos = self.position_of_roomba()
            r.sleep()
            # print format(pos)
            # self.follow_roomba()
            self.follow_roomba_global(des_x=0, des_y=0)

    def test_change_height(self, height=0.0):
        error = height - self.actual_height
        while abs(error) >= 0.5:
            print "error", error
            error = self.change_height(height)
        self.stand_still()

    def test_push_button(self):
        self.test_change_height(height=2.0)
        r = rospy.Rate(10)
        while True:
            angle, is_rotating = self.target_facing_angle()
            r.sleep()
            # print is_rotating
            if not is_rotating or self.should_hit_button:
                if self.check_push_button():
                    self.should_hit_button = True
                else:
                    self.should_hit_button = False

                # Perform push button if applicable
                if self.should_hit_button:
                    self.push_button()
                    self.follow_roomba_global()

                # Otherwise perform back to normal height
                else:
                    self.follow_roomba_global(des_x=0, des_y=-0.5)
                    self.change_height(self.NORMAL_HEIGHT + 0.2)
            else:
                self.should_hit_button = False
                self.follow_roomba_global(des_x=0, des_y=-0.5)
                self.change_height(self.NORMAL_HEIGHT + 0.2)

    def test_land_front(self):
        self.test_change_height(self.NORMAL_HEIGHT)
        r = rospy.Rate(10)

        while True:
            angle, is_rotating = self.target_facing_angle()
            r.sleep()
            # print is_rotating
            if not is_rotating or self.should_hit_button:
                if self.check_push_button():
                    self.should_hit_button = True
                else:
                    self.should_hit_button = False
                    if self.check_land_front():
                        self.should_land_front = True
                    else:
                        self.should_land_front = False

                # Perform push button if applicable
                if self.should_hit_button:
                    self.push_button()
                    self.follow_roomba_global()

                # Perform land in front if applicable
                elif self.should_land_front:
                    self.land_front()

                # Otherwise perform back to normal height
                else:
                    self.follow_roomba_global(des_x=0, des_y=-0.5)
                    self.change_height(self.NORMAL_HEIGHT + 0.2)
            else:
                self.should_hit_button = False
                self.should_land_front = False
                self.follow_roomba_global(des_x=0, des_y=-0.5)
                self.change_height(self.NORMAL_HEIGHT + 0.2)


if __name__ == '__main__':
    rospy.init_node('Example')
    rospy.sleep(0.5)
    roomba = Roomba("target0")
    drone = Drone(target=roomba)

    # Register this behavior (FollowBehavior) to the arbiter
    rospy.Publisher('/arbiter/register', RegisterBehavior, latch=True, queue_size=10).publish(name='follow')

    # This command tell the arbiter that this behavior will take command from now on and abort other behaviors
    rospy.Publisher('/arbiter/activate_behavior', String, latch=True, queue_size=10).publish('follow')

    # drone.test_change_height(drone.NORMAL_HEIGHT)
    # drone.test_follow_roomba()
    # drone.test_push_button()
    drone.test_land_front()

