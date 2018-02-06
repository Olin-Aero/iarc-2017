#!/usr/bin/env/python2
import rospy
from std_msgs.msg import Bool

from Drone import Drone


class Strategy(object):
    def __init__(self):
        rospy.init_node('forebrain')

        self.drone = Drone()
        self.world = WorldState()

    def run(self):
        r = rospy.Rate(20)
        self.world.waitForStart()

        while not self.drone.is_flying():
            self.drone.takeoff(1.5)
            r.sleep()

        self.drone.hover(60)

        self.drone.land()

        rospy.sleep(10)


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
