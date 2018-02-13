#!/usr/bin/env python2
import rospy
from iarc_main.msg import RoombaList, Roomba
from std_msgs.msg import Bool

from Drone import Drone


class Strategy(object):
    def __init__(self):
        rospy.init_node('forebrain')

        self.drone = Drone()
        self.world = WorldState()

        rospy.sleep(1)

    def test_hover(self):
        self.world.waitForStart()

        rospy.loginfo('Taking off')
        self.drone.takeoff(1.5)

        rospy.loginfo('Hovering')
        self.drone.hover(5)

        rospy.loginfo('Landing')
        self.drone.land()

        rospy.loginfo('Done!')

    def test_square(self):
        self.world.waitForStart()

        rospy.loginfo('Taking off')
        self.drone.takeoff(1.5)

        rospy.loginfo('Hovering')
        self.drone.hover(2)

        for target in [(0, 1), (1, 1), (1, 0), (0, 0)]:
            rospy.loginfo('Moving to {}'.format(target))
            self.drone.move_to(target[0], target[1], 'odom')
            rospy.loginfo('Hovering')
            self.drone.hover(2)

        rospy.loginfo('Landing')
        self.drone.land()

    def test_follow(self):
        self.world.waitForStart()
        r = rospy.Rate(20)

        self.drone.takeoff(1.5)
        while not rospy.is_shutdown():
            target = self.choose_target(self.world.targets)
            if target is not None:
                self.drone.move_towards(0, 0, target.frame_id)
            else:
                self.drone.hover(0)
            r.sleep()

    def choose_target(self, targets):
        """
        Selects the most important target
        TODO: Use something more sophisticated
        :param List[Roomba] targets:
        :rtype: Roomba|None
        """
        if len(targets) > 0:
            return targets[0]
        return None

    def run(self):
        self.test_follow()


class WorldState(object):
    def __init__(self):
        self.has_started = False
        self.round_start_time = rospy.Time(0)

        self.targets = []  # type: List[Roomba]
        self.obstacles = []  # type: List[Roomba]

        self.startSub = rospy.Subscriber('start_round', Bool, self.onStart)
        self.roombaSub = rospy.Subscriber('seen_roombas', RoombaList, self.onRoombas)

    def onStart(self, msg):
        if not self.has_started and msg.data:
            self.round_start_time = rospy.Time.now()

        self.has_started = msg.data

    def onRoombas(self, msg):
        """

        :type msg: RoombaList
        """
        targets = []
        obstacles = []
        for roomba in msg.data:  # type: Roomba
            if roomba.type == Roomba.OBSTACLE:
                obstacles.append(roomba)
            else:
                targets.append(roomba)

        self.targets = targets
        self.obstacles = obstacles

        rospy.loginfo_throttle(1, "{} targets and {} obstacles known".format(len(self.targets), len(self.obstacles)))

    def waitForStart(self):
        r = rospy.Rate(50)
        while not self.has_started:
            r.sleep()


if __name__ == '__main__':
    Strategy().run()
