import math
import numpy as np
from numpy import sign

import rospy

import tf.transformations
from tf import TransformListener
from tf.listener import xyz_to_mat44, xyzw_to_mat44

from iarc_arbiter.msg import VelAlt
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class Command:
    """
    This represents the output to the drone
    """
    def __init__(self, vel, land=False, takeoff=False):
        """
        :param Twist vel: Velocity of the drone in vehicle reference frame
        :param bool land: True to cause the drone to automatically land
        :param bool takeoff: True to cause the drone to automatically takeoff
        """
        self.vel = vel
        self.takeoff = takeoff
        self.land = land


def cmd_vel(msg):
    """
    :type msg: Twist
    """
    return Command(msg)


def cmd_takeoff(msg):
    return Command(Twist(), takeoff=True)


def cmd_land(msg):
    return Command(Twist(), land=True)


class PIDAltController(object):
    def __init__(self, tfl, ddynrec):
        """
        :type tfl: TransformListener
        :type ddynrec: DDynamicReconfigure
        """

        self.tf = tfl
        self.config = ddynrec

        self.config.add_variable("kp_height", "Proportional control for altitude adjustment",
                                 rospy.get_param('~kp_height', 0.5), 0.0, 2.0)

        self.config.add_variable("max_vertical_vel", "Maximum speed the drone is allowed to ascend or descend",
                                 rospy.get_param('~max_vertical_vel', 0.5), 0.0, 3.0)

        self.config.add_variable("min_vertical_vel", "Minimum speed the drone is allowed to ascend or descend",
                                 rospy.get_param('~min_vertical_vel', 0.1), 0.0, 3.0)

    def calculate_z_vel(self, error):
        if abs(error) <= 0.05:
            # Good enough! Stop publishing height changing
            return 0

        vel = error * self.config.kp_height

        if abs(vel) < self.config.min_vertical_vel:
            vel = sign(vel) * self.config.min_vertical_vel

        if abs(vel) > self.config.max_vertical_vel:
            vel = sign(vel) * self.config.max_vertical_vel

        return vel

    def get_height(self):
        return self.tf.lookupTransform('odom', 'base_link', rospy.Time(0))[0][2]

    def cmd_vel_alt(self, msg):
        """
        Calculates the commanded velocity given the desired VelAlt message containing a desired altitude and velocity.
        The z-component of the desired velocity is ignored.
        :type msg: VelAlt
        :return: Command
        """

        err = msg.height - self.get_height()

        vel = msg.velocity
        vel.linear.z = self.calculate_z_vel(err)
        return Command(vel)


class PIDPosController(object):
    def __init__(self, tfl, ddynrec, alt_controller):
        """
        :type tfl: TransformListener
        :type ddynrec: DDynamicReconfigure
        :type alt_controller: PIDAltController
        """
        self.tf = tfl
        self.config = ddynrec
        self.alt_controller = alt_controller

        self.last_time = rospy.Time(0)

        self.config.add_variable("max_velocity", "Max velocity the drone can reach",
                                 rospy.get_param('~max_velocity', 1.0), 0.0, 2.0)

        self.config.add_variable("kp_turn", "Proportional Angular",
                                 rospy.get_param('~kp_turn', 0.0), 0.0, 2.0)

        self.config.add_variable("kp", "Proportional Linear",
                                 rospy.get_param('~kp', 0.2), 0.0, .5)

        self.config.add_variable("ki", "Integral",
                                 rospy.get_param('~ki', 0.0), 0.0, .5)

        self.config.add_variable("kd", "Derivative",
                                 rospy.get_param('~kd', 0.2), 0.0, 2.0)

        self.config.add_variable("angle_offset", "Angle the drone should face relative to commanded",
                                 rospy.get_param('~angle_offset', 0.0), -3.14, 3.14)

        self.config.add_variable("use_vel_derivs", "Should  we use the drone's velocity as the derivative terms?",
                                 rospy.get_param('~use_vel_derivs', True))

        self.last_odom = Odometry()
        self.odom_sub = rospy.Subscriber('/odometry', Odometry, self.on_odom)

        self.integral_x = 0.0
        self.previous_error_x = 0.0
        self.integral_y = 0.0
        self.previous_error_y = 0.0

    def on_odom(self, msg):
        """
        :param Odometry msg:
        """
        self.last_odom = msg

    def cmd_pos(self, msg):
        """
        Calculates the commanded velocity given the desired position, with the header of the PoseStamped
        controlling which coordinate frame the commanded position is interpreted as being in.
        :type msg: PoseStamped
        :return: Command
        """

        vel = Twist()

        pose = self.transformPoseFull('base_link', msg, 'odom')
        position = pose.pose.position
        # position = self.tf.transformPose('base_link', msg).pose.position

        # Plan motion
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        # In case something weird happens
        dt = min(dt, 0.5)
        dt = max(dt, 0.01)

        # Calculate DIP velocity_x
        error_x = position.x
        self.integral_x = self.integral_x + error_x * dt

        if self.config.use_vel_derivs:
            derivative_x = -self.last_odom.twist.twist.linear.x
        else:
            derivative_x = (error_x - self.previous_error_x) / dt

        self.previous_error_x = error_x
        dip_x = self.config.kp * error_x + self.config.ki * self.integral_x + self.config.kd * derivative_x

        # Calculate DIP velocity_y
        error_y = position.y
        self.integral_y = self.integral_y + error_y * dt

        if self.config.use_vel_derivs:
            derivative_y = -self.last_odom.twist.twist.linear.y
        else:
            derivative_y = (error_y - self.previous_error_y) / dt

        self.previous_error_y = error_y
        dip_y = self.config.kp * error_y + self.config.ki * self.integral_y + self.config.kd * derivative_y

        # Combined velocity
        dip_diagonal = math.sqrt(dip_x ** 2 + dip_y ** 2)

        if dip_diagonal < self.config.max_velocity:
            # If dip_diagonal is already < max_velocity
            # use dip velocity
            vel.linear.x = dip_x
            vel.linear.y = dip_y
        else:
            # Else use max velocity
            diagonalvelocity = math.sqrt(position.x ** 2 + position.y ** 2)
            vel.linear.x = position.x / diagonalvelocity * self.config.max_velocity
            vel.linear.y = position.y / diagonalvelocity * self.config.max_velocity

            # Reset integral until we start using it
            self.integral_x = 0.0
            self.integral_y = 0.0

        # Preserve the z value of velocity
        vel.linear.z = self.alt_controller.calculate_z_vel(position.z)

        # Turn drone toward the provided orientation
        _, _, angle_err = tf.transformations.euler_from_quaternion(
            [getattr(pose.pose.orientation, s) for s in 'xyzw'])
        angle_err += self.config.angle_offset

        # Normalize the angle
        while angle_err <= -np.pi:
            angle_err += 2 * np.pi
        while angle_err > np.pi:
            angle_err -= 2 * np.pi

        vel.angular.z = self.config.kp_turn * angle_err

        print("Calculated vel: {}".format(vel))

        return Command(vel)

    def transformPoseFull(self, target_frame, ps, stationary):
        """
        Transforms a geometry_msgs PoseStamped message to frame target_frame, assuming that stationary is a
        stationary frame.

        Only useful in cases where ps.header.stamp == rospy.Time(0)

        Adapted from:
            https://github.com/ros/geometry/blob/indigo-devel/tf/src/tf/listener.py#L286

        :param stationary: the tf frame the point will be transformed through
        :param target_frame: the tf target frame, a string
        :param ps: the geometry_msgs.msg.PoseStamped message
        :return: new geometry_msgs.msg.PoseStamped message, in frame target_frame
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise

        """
        # mat44a is frame-to-frame transform from the origin frame to the stationary frame
        mat44a = self.tf.asMatrix(stationary, ps.header)

        # mat44b is a frame-to-frame transform from the stationary frame to the target frame
        intermediate_header = Header(frame_id=stationary, stamp=rospy.Time(0))
        mat44b = self.tf.asMatrix(target_frame, intermediate_header)

        # pose44 is the given pose as a 4x4
        pose44 = np.dot(xyz_to_mat44(ps.pose.position), xyzw_to_mat44(ps.pose.orientation))

        # tx_pose is the new pose in target_frame as a 4x4
        tx_pose = np.dot(np.dot(mat44b, mat44a), pose44)

        # xyz and quat are tx_pose's position and orientation
        xyz = tuple(tf.transformations.translation_from_matrix(tx_pose))[:3]
        quat = tuple(tf.transformations.quaternion_from_matrix(tx_pose))

        # assemble return value PoseStamped
        result = PoseStamped()
        result.header.stamp = ps.header.stamp
        result.header.frame_id = target_frame
        result.pose = Pose(Point(*xyz), Quaternion(*quat))
        return result


if __name__ == '__main__':
    test_msg = PoseStamped()
    test_msg.pose.position.x = 1
    test_msg.pose.position.y = 1
    test_msg.pose.position.z = 0
    test_msg.header.frame_id = "target0"

    rospy.init_node('transformer_test')
    r = rospy.Rate(10)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    controller = PIDPosController(TransformListener())
    rospy.sleep(0.5)
    while not rospy.is_shutdown():
        r.sleep()
        pub.publish(controller.cmd_pos(test_msg).vel)
