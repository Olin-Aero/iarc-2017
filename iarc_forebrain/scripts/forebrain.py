#!/usr/bin/env python2
import rospy
from std_msgs.msg import Bool

from Drone import Drone


class Strategy(object):
    def __init__(self):
        rospy.init_node('forebrain')

        self.drone = Drone()
        self.world = WorldState()

    def test_hover(self):
        r = rospy.Rate(20)
        self.world.waitForStart()

        rospy.loginfo('Taking off')
        while not self.drone.is_flying():
            self.drone.takeoff(1.5)
            r.sleep()

        rospy.loginfo('Hovering')
        self.drone.hover(5)

        rospy.loginfo('Landing')
        self.drone.land()

        rospy.loginfo('Done!')
        rospy.sleep(5)

    def test_square(self):
        r = rospy.Rate(20)
        self.world.waitForStart()

        rospy.loginfo('Taking off')
        while not self.drone.is_flying():
            self.drone.takeoff(1.5)
            r.sleep()

        rospy.loginfo('Hovering')
        self.drone.hover(2)

        rospy.loginfo('Moving 1')
        self.drone.move_to(0, 1, 'odom')

        rospy.loginfo('Moving 2')
        self.drone.move_to(1, 1, 'odom')

        rospy.loginfo('Moving 3')
        self.drone.move_to(1, 0, 'odom')

        rospy.loginfo('Moving 4')
        self.drone.move_to(0, 0, 'odom')

        self.drone.land()

        rospy.sleep(3)

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
