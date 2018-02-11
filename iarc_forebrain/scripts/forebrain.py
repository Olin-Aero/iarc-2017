#!/usr/bin/env python2
import rospy
from std_msgs.msg import Bool

from Drone import Drone


class Strategy(object):
    def __init__(self):
        rospy.init_node('forebrain')

        self.drone = Drone()
        self.world = WorldState()

        rospy.sleep(1)

    def test_hover(self):
        r = rospy.Rate(20)
        self.world.waitForStart()

        rospy.loginfo('Taking off')
        self.drone.takeoff(1.5)

        rospy.loginfo('Hovering')
        self.drone.hover(5)

        rospy.loginfo('Landing')
        self.drone.land()

        rospy.loginfo('Done!')

    def test_square(self):
        r = rospy.Rate(20)
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

    def run(self):
        self.test_square()


class WorldState(object):
    def __init__(self):
        self.has_started = False
        self.round_start_time = rospy.Time(0)

        self.startSub = rospy.Subscriber('start_round', Bool, self.onStart)

    def onStart(self, msg):
        if not self.has_started and msg.data:
            self.round_start_time = rospy.Time.now()

        self.has_started = msg.data

    def waitForStart(self):
        r = rospy.Rate(10)
        while not self.has_started:
            r.sleep()


if __name__ == '__main__':
    Strategy().run()
