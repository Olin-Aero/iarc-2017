from geometry_msgs.msg import Twist, PoseStamped


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


def cmd_pos(msg):
    """
    :type msg: PoseStamped
    :return: Command
    """

    vel = Twist()
    # does stuff

    return Command(vel)
