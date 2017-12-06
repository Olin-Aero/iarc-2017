import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from tf import TransformListener
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


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

        # Testing follow behavior at position (x, y) = (1, 1) relative to target
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

        # self.maxvelocity = rospy.get_param('~max_velocity', 1.0)  # Max velocity the drone can reach
        # self.kpturn = rospy.get_param('~kp_turn', 0.0)  # Proportional angular for turning
        # self.kp = rospy.get_param('~kp', 0.05)  # Proportional linear
        # self.ki = rospy.get_param('~ki', 0)#0.02)  # Integral linear
        # self.kd = rospy.get_param('~kd', 0.1)  # Derivative linear

        self.integral_x = 0.0
        self.previous_error_x = 0.0
        self.integral_y = 0.0
        self.previous_error_y = 0.0

    def cmd_pos(self, msg):
        """
        :type msg: PoseStamped
        :return: Command
        """

        vel = Twist()

        position = self.tf.transformPose('base_link', msg).pose.position

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
        integral_y = self.integral_y + error_y * dt
        derivative_y = (error_y - self.previous_error_y) / dt
        self.previous_error_y = error_y
        dip_y = self.config.kp * error_y + self.config.ki * integral_y + self.config.kd * derivative_y

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

        # Turn drone to where it's heading to
        vel.angular.z = self.config.kp_turn * math.atan2(vel.linear.y, vel.linear.x)

        print("Calculated vel: {}".format(vel))

        return Command(vel)


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
