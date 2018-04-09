#!/usr/bin/env python2
import rospy
from iarc_main.msg import RoombaList, Roomba
from iarc_strategy.srv import ExplorationTarget, ExplorationTargetRequest, ExplorationTargetResponse

from std_msgs.msg import Bool
from ChooseTarget import goodnessScore, targetSelect
from Drone import Drone

from tf import TransformListener
import tf.transformations

from math import pi


class Strategy(object):
    def __init__(self):
        rospy.init_node('forebrain')

        tfl = TransformListener()
        self.drone = Drone(tfl=tfl)
        self.world = WorldState(tfl=tfl)
        self._explore_srv = rospy.ServiceProxy('/explorer/explore', ExplorationTarget)

        rospy.sleep(1)
        
    def test_hover(self):
        self.world.wait_for_start()

        rospy.loginfo('Taking off')
        self.drone.takeoff(1.5)

        rospy.loginfo('Hovering')
        self.drone.hover(5)

        rospy.loginfo('Landing')
        self.drone.land()

        rospy.loginfo('Done!')

    def test_square(self):
        self.world.wait_for_start()

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
        #self.world.wait_for_start()
        r = rospy.Rate(20)
        self.drone.takeoff(1.5)
        while not rospy.is_shutdown():
            target = self.choose_target(self.world.targets)
            if target is not None:
                rospy.loginfo('Current Target : {}'.format(target))
                self.drone.move_towards(0, 0, target.frame_id)
            else:
                rospy.loginfo('explore')
                resp = self._explore_srv()
                if resp.success:
                    target = resp.target
                    rospy.loginfo('Exploration Target : {}'.format(target))
                    self.drone.move_to(des_x=target.x, des_y=target.y, frame='map', height=target.z)
                else:
                    # fallback
                    self.drone.hover(0)
            r.sleep()

    def test_follow_redirect(self):
        #self.world.wait_for_start()
        r = rospy.Rate(20)
        self.drone.takeoff(1.5)
        while not rospy.is_shutdown():
            target = self.choose_target(self.world.targets)
            if target is not None:
                angleDiff = abs(angle_diff(self.world.target_facing_angle(target), self.world.CORRECT_DIRECTION))
                if angleDiff > pi/2:
                    success = self.drone.redirect_180(target)
                    if success:
                        rospy.loginfo('Redirected roomba: Success!')
                    else:
                        rospy.loginfo('Redirected roomba: Failure :(')
                # elif angleDiff <= 3*pi/4 and angleDiff >= pi/4:
                #     self.drone.redirect_45(target)
                else:
                    # Follow the roomba
                    self.drone.move_towards(0, 0, target.frame_id, 2.5)
            else:
                resp = self._explore_srv()
                if resp.success:
                    target = resp.target
                    rospy.loginfo('Exploration Target : {}'.format(target))
                    self.drone.move_to(des_x=target.x, des_y=target.y, frame='map', height=target.z)
                else:
                    # fallback
                    self.drone.hover(0, 1.5)
            r.sleep()

    def test_explore(self):
        while not rospy.is_shutdown():
            resp = self._explore_srv()
            if resp.success:
                target = resp.target
                rospy.loginfo('Exploration Target : {}'.format(target))
                self.drone.move_to(des_x=target.x, des_y=target.y, frame='map', height=target.z)
            else:
                # fallback
                self.drone.hover(0, 1.5)

    def choose_target(self, targets):
        """
        Selects the most important target
        TODO: Use something more sophisticated
        :param List[Roomba] targets:
        :rtype: Roomba|None
        """
        s = targetSelect(goodnessScore(targets))
        if(s[1] < -100):
            return None
        return s[0]
        # closestRoomba = None
        # minimumCloseness = pi
        # for i in targets:
        #     position, quaternion = self.tfl.lookupTransform("map", i.frame_id, rospy.Time(0))
        #     heading = tf.transformations.euler_from_quaternion(quaternion)
        #     closeToPiOver2 = abs(heading[2] % pi - pi / 2)
        #     if(closeToPiOver2 < minimumCloseness):
        #         minimumCloseness = closeToPiOver2
        #         closestRoomba = i
        # return closestRoomba

    def run(self):
        self.test_explore()
        #self.test_follow()


def angle_diff(a, b):
    """Returns x=a-b, wrapped to be -pi<x<=pi"""
    diff = a - b
    while diff <= -pi:
        diff += 2 * pi
    while diff > pi:
        diff -= 2 * pi

    return diff


class WorldState(object):
    """
    This python object (not a ROS node) helps subscribe to and track
    various things about the match and world external to the drone.
    It is, in effect, a collection of helper functions to make writing
    strategies easier.
    """
    CORRECT_DIRECTION = pi/2
    def __init__(self, tfl=None):
        self.has_started = False
        self.round_start_time = rospy.Time(0)

        self.targets = []  # type: List[Roomba]
        self.obstacles = []  # type: List[Roomba]

        if tfl is None:
            self.tfl = TransformListener()
        else:
            self.tfl = tfl

        self.startSub = rospy.Subscriber('start_round', Bool, self._on_start)
        self.roombaSub = rospy.Subscriber('seen_roombas', RoombaList, self._on_roombas)

    def _on_start(self, msg):
        if not self.has_started and msg.data:
            self.round_start_time = rospy.Time.now()

        self.has_started = msg.data

    def _on_roombas(self, msg):
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

    def wait_for_start(self):
        r = rospy.Rate(50)
        while not self.has_started:
            r.sleep()

    def round_phase(self):
        dt = rospy.Time.now() - self.round_start_time

        return (dt.to_sec() / 20) % 1.0

    def target_facing_angle(self, roomba):
        """
        :param Roomba roomba: the Roomba we are interested in
        :return float: The angle the target is facing from -PI to PI, where pi/2 is towards green, and -pi/2 is towards red
        """

        position, quaternion = self.tfl.lookupTransform("map", roomba.frame_id, rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(quaternion)

        return euler[-1]


if __name__ == '__main__':
    Strategy().run()
