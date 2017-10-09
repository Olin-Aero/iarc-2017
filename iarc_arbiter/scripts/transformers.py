import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from tf import TransformListener

cmd_vel_multiplier = 0.3


class Command:
    def __init__(self, vel, land=False, takeoff=False):
        self.vel = vel
        self.takeoff = takeoff
        self.land = land


def cmd_vel(msg):
    """
    :type msg: Twist
    """
    vel = msg
    vel.linear.x *= cmd_vel_multiplier
    vel.linear.y *= cmd_vel_multiplier
    return Command(vel)


def cmd_vel_raw(msg):
    """
    :type msg: Twist
    """
    return Command(msg)


def cmd_takeoff(msg):
    return Command(Twist(), takeoff=True)


def cmd_land(msg):
    return Command(Twist(), land=True)


def cmd_pos(msg):
    """
    :type msg: PoseStamped
    :return: Command
    """

    vel = Twist()

    position = tf.transformPose('base_link', msg).pose.position

    global last_time
    global integral_x
    global previous_error_x
    global integral_y
    global previous_error_y

    # Plan motion
    now = rospy.Time.now()
    dt = (now - last_time).to_sec()
    last_time = now

    # In case something weird happens
    dt = min(dt, 0.5)
    dt = max(dt, 0.01)

    # Calculate DIP velocity_x
    error_x = position.x
    integral_x = integral_x + error_x * dt
    derivative_x = (error_x - previous_error_x) / dt
    previous_error_x = error_x
    dip_x = kp * error_x + ki * integral_x + kd * derivative_x

    # Calculate DIP velocity_y
    error_y = position.y
    integral_y = integral_y + error_y * dt
    derivative_y = (error_y - previous_error_y) / dt
    previous_error_y = error_y
    dip_y = kp * error_y + ki * integral_y + kd * derivative_y

    # Combined velocity
    dip_diagonal = math.sqrt(dip_x ** 2 + dip_y ** 2)

    if dip_diagonal < maxvelocity:
        # If dip_diagonal is already < max_velocity
        # use dip velocity
        vel.linear.x = dip_x
        vel.linear.y = dip_y
    else:
        # Else use max velocity
        diagonalvelocity = math.sqrt(position.x ** 2 + position.y ** 2)
        vel.linear.x = position.x / diagonalvelocity * maxvelocity
        vel.linear.y = position.y / diagonalvelocity * maxvelocity

        # Reset integral until we start using it
        integral_x = 0.0
        integral_y = 0.0

    # Turn drone to where it's heading to
    vel.angular.z = kpturn * math.atan2(vel.linear.y, vel.linear.x)

    print("Calculated vel: {}".format(vel))

    return Command(vel)


# Testing follow behavior at position (x, y) = (1, 1) relative to target
tf = TransformListener()
last_time = rospy.Time(0)

maxvelocity = rospy.get_param('~max_velocity', 1.0)  # Max velocity the drone can reach
kpturn = rospy.get_param('~kp_turn', 1.0)  # Proportional for turning
kp = rospy.get_param('~kp', 1.0)  # Proportional
ki = rospy.get_param('~ki', 0.2)  # Integral
kd = rospy.get_param('~kd', 0.0)  # Derivative: kd is not currently used

integral_x = 0.0
previous_error_x = 0.0
integral_y = 0.0
previous_error_y = 0.0

msg = PoseStamped()
msg.pose.position.x = 1
msg.pose.position.y = 1
msg.pose.position.z = 0
msg.header.frame_id = "target0"

if __name__ == '__main__':
    rospy.init_node('transformer_test')
    r = rospy.Rate(10)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    while not rospy.is_shutdown():
        r.sleep()
        pub.publish(cmd_pos(msg).vel)

rospy.sleep(1.0)
