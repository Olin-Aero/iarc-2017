import math
import numpy as np
from numpy import sign

import rospy
import tf.transformations
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from tf import TransformListener
from tf.listener import xyz_to_mat44, xyzw_to_mat44


class Command:
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


class PIDController(object):
    def __init__(self, tf, ddynrec):
        """
        :type tf: TransformListener
        :type ddynrec: DDynamicReconfigure
        """
        self.tf = tf
        self.config = ddynrec

        self.last_time = rospy.Time(0)

        self.config.add_variable("max_velocity", "Max velocity the drone can reach",
                                 rospy.get_param('~max_velocity', 1.0), 0.0, 2.0)

        self.config.add_variable("kp_turn", "Proportional Angular",
                                 rospy.get_param('~kp_turn', 0.0), 0.0, 2.0)

        self.config.add_variable("kp", "Proportional Linear",
                                 rospy.get_param('~kp', 0.3), 0.0, .5)

        self.config.add_variable("ki", "Integral",
                                 rospy.get_param('~ki', 0.0), 0.0, .5)

        self.config.add_variable("kd", "Derivative",
                                 rospy.get_param('~kd', 0.3), 0.0, 2.0)

        self.config.add_variable("kp_height", "Proportional control for altitude adjustment",
                                 rospy.get_param('~kp_height', 0.5), 0.0, 2.0)

        self.config.add_variable("max_vertical_vel", "Maximum speed the drone is allowed to ascend or descend",
                                 rospy.get_param('~max_vertical_vel', 0.5), 0.0, 3.0)

        self.config.add_variable("min_vertical_vel", "Minimum speed the drone is allowed to ascend or descend",
                                 rospy.get_param('~min_vertical_vel', 0.1), 0.0, 3.0)

        # self.maxvelocity = rospy.get_param('~max_velocity', 1.0)  # Max velocity the drone can reach
        # self.kpturn = rospy.get_param('~kp_turn', 0.0)  # Proportional angular for turning
        # self.kp = rospy.get_param('~kp', 0.05)  # Proportional linear
        # self.ki = rospy.get_param('~ki', 0)#0.02)  # Integral linear
        # self.kd = rospy.get_param('~kd', 0.1)  # Derivative linear

        # self.kp_height = 1.0
        # self.MAX_VERTICAL_VEL = 1.5
        # self.MIN_VERTICAL_VEL = 0.2

        self.integral_x = 0.0
        self.previous_error_x = 0.0
        self.integral_y = 0.0
        self.previous_error_y = 0.0

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

    def cmd_pos(self, msg):
        """
        Calculates the commanded velocity given the desired position, with the header of the PoseStamped
        controlling which coordinate frame the commanded position is interpreted as being in.
        :type msg: PoseStamped
        :return: Command
        """

        vel = Twist()

        position = self.transformPoseFull('base_link', msg, 'odom').pose.position
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
        derivative_x = (error_x - self.previous_error_x) / dt
        self.previous_error_x = error_x
        dip_x = self.config.kp * error_x + self.config.ki * self.integral_x + self.config.kd * derivative_x

        # Calculate DIP velocity_y
        error_y = position.y
        self.integral_y = self.integral_y + error_y * dt
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
        vel.linear.z = self.calculate_z_vel(position.z)

        # Turn drone to where it's heading to
        vel.angular.z = self.config.kp_turn * math.atan2(vel.linear.y, vel.linear.x)

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
        r = PoseStamped()
        r.header.stamp = ps.header.stamp
        r.header.frame_id = target_frame
        r.pose = Pose(Point(*xyz), Quaternion(*quat))
        return r


if __name__ == '__main__':
    test_msg = PoseStamped()
    test_msg.pose.position.x = 1
    test_msg.pose.position.y = 1
    test_msg.pose.position.z = 0
    test_msg.header.frame_id = "target0"

    rospy.init_node('transformer_test')
    r = rospy.Rate(10)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    controller = PIDController(TransformListener())
    rospy.sleep(0.5)
    while not rospy.is_shutdown():
        r.sleep()
        pub.publish(controller.cmd_pos(test_msg).vel)

