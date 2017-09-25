from geometry_msgs.msg import Twist

cmd_vel_multiplier = 0.3


def cmd_vel(msg):
    """
    :type msg: Twist
    """
    msg.linear.x *= cmd_vel_multiplier
    msg.linear.y *= cmd_vel_multiplier
    return Command(msg)


def raw_cmd_vel(msg):
    """
    :type msg: Twist
    """
    return Command(msg)


class Command:
    def __init__(self, vel, land=False, takeoff=False):
        self.vel = vel
        self.takeoff = takeoff
        self.land = land
