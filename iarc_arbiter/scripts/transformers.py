from geometry_msgs.msg import Twist

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
    msg.linear.x *= cmd_vel_multiplier
    msg.linear.y *= cmd_vel_multiplier
    return Command(msg)


def cmd_vel_raw(msg):
    """
    :type msg: Twist
    """
    return Command(msg)


def cmd_takeoff(msg):
    return Command(Twist(), takeoff=True)


def cmd_land(msg):
    return Command(Twist(), land=True)
