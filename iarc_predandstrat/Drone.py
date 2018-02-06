import math
import rospkg

import rospy
from numpy import sign, pi
import sys
import os
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from iarc_arbiter.msg import RegisterBehavior
from std_msgs.msg import Float64, String
import tf, tf.transformations

from iarc_main.msg import Roomba

rospack = rospkg.RosPack()
iarc_sim_path = rospack.get_path('iarc_sim_2d')
sys.path.append(os.path.join(iarc_sim_path, 'src'))

import config as cfg


class Drone:
    def __init__(self, target=None):
        self.should_hit_button = False
        self.should_land_front = False
        self.vel3d = Twist()
        self.MAX_VERTICAL_VEL = 2.0
        self.MIN_VERTICAL_VEL = 0.8
        self.NORMAL_HEIGHT = 2.5

        self.height_target = 0.0

        self.current_target = target  # Roomba class
        self.prev_target_facing_angle = None  # in radian

        self.FRAME_ID = "base_link"
        self.tf = tf.TransformListener()
        self.followPosPub = rospy.Publisher('/follow/cmd_pos', PoseStamped, queue_size=10)
        self.followVelPub = rospy.Publisher('/follow/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber("/cmd_vel", Twist, self.record_vel)

    def record_vel(self, msg):
        # TODO: Use Odometry topic for velocity information
        self.vel3d = msg

    def follow_roomba(self, roomba=None, des_x=0.0, des_y=0.0, des_z=None):
        """
        Follow roomba X with desired position des_x, des_y
        :param roomba: an roomba object
        :param des_x: desired position x
        :param des_y: desired position y
        :param des_z: desired position z
        """
        if roomba is None:
            if self.current_target is None:
                return None
            roomba = self.current_target

        if des_z is not None:
            self.height_target = des_z

        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = des_x
        pose_stamped.pose.position.y = des_y
        pose_stamped.pose.position.z = self.height_target
        pose_stamped.header.frame_id = roomba.frame_id

        self.followPosPub.publish(pose_stamped)

    def follow_roomba_global(self, des_x=0.0, des_y=0.0, des_z=None):
        """
        Follow roomba X with desired position des_x, des_y, ignore roomba's orientation
        :param des_x: desired position x
        :param des_y: desired position y
        :param des_z: desired position z
        """
        if self.current_target is None:
            return None

        position = self.position_of_roomba()

        if des_z is not None:
            self.height_target = position[2] + des_z

        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = position[0] + des_x
        pose_stamped.pose.position.y = position[1] + des_y
        pose_stamped.pose.position.z = self.height_target
        pose_stamped.header.frame_id = "map"

        self.followPosPub.publish(pose_stamped)

    def move(self, des_x=0.0, des_y=0.0, des_z=None):
        """
        Tell the drone to move relative to itself, going to the height target if des_z
        is not specified
        :param des_x: desired position x
        :param des_y: desired position y
        :param des_z: desired position z
        """

        if des_z is None:
            des_z = self.height_target - self.height()

        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = des_x
        pose_stamped.pose.position.y = des_y
        pose_stamped.pose.position.z = des_z
        pose_stamped.header.frame_id = self.FRAME_ID

        self.followPosPub.publish(pose_stamped)

    def change_height(self, height):
        """
        Change drone's height to height
        :param height: Desired height, in meters
        """
        self.height_target = height
        self.move()

    def stand_still(self):
        """
        Make our drone stand still
        NOTE: commands zero velocity, overriding vertical height setpoints
        :return: None
        """
        self.followVelPub.publish(Twist())

    def push_button(self):
        """
        Make our drone push button. Go together with a boolean should_hit_button
        should_hit_button is True, execute this method
        Otherwise: do nothing
        :return: Void
        """
        if self.distance_from_target() is None:
            print "No target to hit button"
            self.should_hit_button = False

        self.follow_roomba_global()
        if self.should_hit_button:
            # Hit the button
            self.change_height(cfg.ROOMBA_HEIGHT - 0.1)
            if self.distance_from_target() < 0.1:

                # 0.1 is roomba's height
                if self.height() <= cfg.ROOMBA_HEIGHT:
                    print "Button hit"
                    self.should_hit_button = False
                    self.change_height(self.NORMAL_HEIGHT)

    def land_front(self):
        """
        Make our drone land in front of the current target. Go together with a boolean should_land_front
        should_land_front is True, execute this method
        Otherwise: do nothing
        :return: Void
        """
        if self.distance_from_target() is None:
            print "No target to stand in front"
            self.should_land_front = False

        if self.should_land_front:
            # Land in front
            if self.vel3d.linear.x != 0 or self.vel3d.linear.y != 0:
                # We are moving
                self.follow_roomba_global(des_y=-0.6, des_z=0.0)

            elif self.height() <= 0.1:
                if self.distance_from_target() <= 0.35:
                    print "Roomba hit! Now,taking off"
                    self.should_land_front = False
                    self.follow_roomba_global(des_y=-0.6, des_z=0.0)
                elif self.distance_from_target() <= 0.8:
                    print "Landed in front, waiting for roomba"
                    self.stand_still()

    def distance_from_target(self):
        """
        Get the distance from the target relative to drone
        :return: Float: distance
        """
        if self.current_target is None:
            return None

        position, quaternion = self.tf.lookupTransform(self.FRAME_ID, self.current_target.frame_id, rospy.Time(0))
        distance = math.sqrt(position[0] ** 2 + position[1] ** 2)
        return distance

    def height(self):
        position, quaternion = self.tf.lookupTransform('map', self.FRAME_ID, rospy.Time(0))
        return position[2]

    def position_of_roomba(self):
        """
        Gets the relative position of the current target to the drone
        :return: List[float]: relative coordinates
        """
        if self.current_target is None:
            return None

        position, quaternion = self.tf.lookupTransform("map", self.current_target.frame_id, rospy.Time(0))
        return position

    def target_facing_angle(self):
        """
        :return: Float: The angle the target is facing from -PI to PI
        :return: True: if target is rotating, False: otherwise
        """
        if self.current_target is None:
            return None

        position, quaternion = self.tf.lookupTransform("map", self.current_target.frame_id, rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(quaternion)
        is_rotating = euler[-1] != self.prev_target_facing_angle
        # print euler[-1] / pi * 180
        self.prev_target_facing_angle = euler[-1]
        return euler[-1], is_rotating

    def check_back_to_height(self):
        """
        Check if our drone is back to normal height
        :return: True: if already back to normal height, False: otherwise
        """
        if self.height() <= self.NORMAL_HEIGHT:
            return False
        # print "Back to normal height"
        return True

    def check_push_button(self):
        """
        Check if the target is facing to the right direction
        If the target is heading to a wrong direction, drone should push button
        :return: True: need to push button, False: otherwise
        """
        angle, is_rotating = self.target_facing_angle()
        if angle is None or is_rotating:
            return False
        return -pi / 3 <= angle <= pi / 3 or angle >= 2 * pi / 3 or angle <= -2 * pi / 3

    def check_land_front(self):
        """
        Check if the target is heading towards Red region, angle from -pi/4 to -3 pi/4
        If the target is heading to Red region, drone should land in front
        :return: True: need to land in front, False: otherwise
        """
        angle, is_rotating = self.target_facing_angle()
        if angle is None or is_rotating:
            return False

        return -2 * pi / 3 < angle < -pi / 3

    def decision_making(self):
        """
        Three decisions currently: push button -> land in front -> just follow
        :return: Void
        """
        if self.check_push_button():
            self.should_hit_button = True
        else:
            self.should_hit_button = False
            if self.check_land_front():
                self.should_land_front = True
            else:
                self.should_land_front = False

    """
    These methods are for testing purposes
    """

    def test_follow_roomba(self):
        self.test_change_height(3)
        r = rospy.Rate(10)

        while True:
            r.sleep()
            self.follow_roomba_global(des_x=0, des_y=0)

    def test_change_height(self, height=0.0):
        error = height - self.height()
        r = rospy.Rate(10)

        while abs(error) >= 0.5:
            r.sleep()
            print "error", error
            self.change_height(height)
            error = height - self.height()
        self.stand_still()

    def test_push_button(self):
        self.test_change_height(height=2.0)
        r = rospy.Rate(10)

        while True:
            angle, is_rotating = self.target_facing_angle()
            r.sleep()

            if not is_rotating or self.should_hit_button:
                if self.check_push_button():
                    self.should_hit_button = True
                else:
                    self.should_hit_button = False

                # Perform push_button if applicable
                if self.should_hit_button:
                    self.push_button()
                    self.follow_roomba_global()

                # Otherwise perform back_to_normal_height
                else:
                    self.follow_roomba_global(des_x=0, des_y=-0.5, des_z=self.NORMAL_HEIGHT)
            else:
                self.should_hit_button = False
                self.follow_roomba_global(des_x=0, des_y=-0.5, des_z=self.NORMAL_HEIGHT)

    def test_land_front_n_hit_btn(self):
        self.test_change_height(self.NORMAL_HEIGHT)
        r = rospy.Rate(10)

        while True:
            angle, is_rotating = self.target_facing_angle()
            r.sleep()

            if not is_rotating or self.should_hit_button:
                self.decision_making()

                # Perform push_button if applicable
                if self.should_hit_button:
                    self.push_button()

                # Perform land_in_front if applicable
                elif self.should_land_front:
                    self.land_front()

                # Otherwise perform back_to_normal_height
                else:
                    self.follow_roomba_global(des_x=0, des_y=-0.5, des_z=self.NORMAL_HEIGHT)
            else:
                self.should_hit_button = False
                self.should_land_front = False
                self.follow_roomba_global(des_x=0, des_y=-0.5, des_z=self.NORMAL_HEIGHT)


if __name__ == '__main__':
    rospy.init_node('Example')
    rospy.sleep(0.5)
    roomba = Roomba("target4")
    drone = Drone(target=roomba)

    # Register this behavior (FollowBehavior) to the arbiter
    rospy.Publisher('/arbiter/register', RegisterBehavior, latch=True, queue_size=10).publish(name='follow')

    # This command tell the arbiter that this behavior will take command from now on and abort other behaviors
    rospy.Publisher('/arbiter/activate_behavior', String, latch=True, queue_size=10).publish('follow')

    # drone.test_change_height(drone.NORMAL_HEIGHT)
    # drone.test_follow_roomba()
    # drone.test_push_button()
    drone.test_land_front_n_hit_btn()