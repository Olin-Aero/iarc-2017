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
from iarc_main.msg import StartRound
from geometry_msgs.msg import PoseStamped


class Strategy(object):
    def __init__(self):
        rospy.init_node('forebrain')

        tfl = TransformListener()
        self.drone = Drone(tfl=tfl)
        self.world = WorldState(tfl=tfl)
        self._explore_srv = rospy.ServiceProxy('/explorer/explore', ExplorationTarget)
        self._tfl = tfl
        self.targeting = None
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

        last_explore = None
        explore_target = None

        while not rospy.is_shutdown():
            target = self.choose_target(self.world.targets, self.world.obstacles)
            if target is not None:
                rospy.loginfo('Current Target : {}'.format(target))
                self.drone.move_towards(des_x=target.x, des_y=target.y, frame='map', height=target.z)
                #self.drone.move_towards(0, 0, target.frame_id)
                #self.drone.move_towards(0, 0, target.frame_id)
            else:
                now = rospy.Time.now()
                if last_explore is None or (now - last_explore).to_sec() > 5.0: 
                    rospy.loginfo('explore')
                    resp = self._explore_srv()
                    if resp.success:
                        last_explore = now
                        explore_target = resp.target
                        rospy.loginfo('Exploration Target : {}'.format(target))
                    else:
                        explore_target = None
                    #self.drone.move_to(des_x=target.x, des_y=target.y, frame='map', height=target.z)

                if explore_target is None:
                    # fallback
                    self.drone.hover(0)
                else:
                    self.drone.move_towards(des_x=explore_target.x, des_y=explore_target.y, frame='map', height=explore_target.z)
            r.sleep()

    def test_follow_redirect(self):
        #self.world.wait_for_start()
        r = rospy.Rate(20)
        self.drone.takeoff(1.5)

        while not rospy.is_shutdown():
            target = self.choose_target(self.world.targets,self.world.obstacles)
            if target is not None:
                angleDiff = angle_diff(self.world.target_facing_angle(target), self.world.CORRECT_DIRECTION)
                if(angleDiff < 3 * pi / 4 and angleDiff > pi / 4):
                    success = self.drone.redirect_45(target)
                elif abs(angleDiff) > pi/2:
                    success = self.drone.redirect_180(target)
                    if success:
                        rospy.loginfo('Redirected roomba: Success!')
                    else:
                        rospy.loginfo('Redirected roomba: Failure :(')
                else:
                    # Follow the roomba
                    self.drone.move_towards(0, 0, target.frame_id, 2.5)
            else:
                now = rospy.Time.now()


                resp = self._explore_srv()
                if resp.success:
                    target = resp.target
                    rospy.loginfo('Exploration Target : {}'.format(target))
                    self.drone.move_to(des_x=target.x, des_y=target.y, frame='map', height=target.z)
                else:
                    fallback
                self.drone.hover(0, 1.5)
            r.sleep()

    def test_explore(self):
        #rosrun iarc_strategy explorer.py _map_frame:=odom
        while not rospy.is_shutdown():
            resp = self._explore_srv()
            if resp.success:
                target = resp.target
                rospy.loginfo('Exploration Target : {}'.format(target))
                self.drone.move_to(des_x=target.x, des_y=target.y, frame='map', height=target.z)
            else:
                # fallback
                self.drone.hover(0, 1.5)

    def choose_target(self, targets, obstacles):
        """
        Selects the most important target
        TODO: Use something more sophisticated
        :param List[Roomba] targets:
        :rtype: Roomba|None
        """
        #s = targetSelect(goodnessScore(targets, obstacles, self.targeting))
        #s = targets[-1]
        #if targets != []:
        #    s = (targets[-1], 0)
        #else:
        #    return None

        ##if(s[1] < -100):
        ##    return None
        #self.targeting = s[0]
        #return s[0]

        closestRoomba = None
        minimumCloseness = pi
        for i in targets:
            #position, quaternion = self.tfl.lookupTransform("map", i.frame_id, rospy.Time(0))
            ps = PoseStamped(
                    header = i.visible_location.header,
                    pose = i.visible_location.pose.pose
                    )
            try:
                map_ps = self._tfl.transformPose('map', ps)
            except tf.Exception as e:
                rospy.loginfo_throttle(0.5, 'map_ps failed : {}'.format(e))
                continue
            position, quaternion = map_ps.pose.position, map_ps.pose.orientation
            quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

            heading = tf.transformations.euler_from_quaternion(quaternion)
            closeToPiOver2 = abs(heading[2] % pi - pi / 2)
            if(closeToPiOver2 < minimumCloseness):
                minimumCloseness = closeToPiOver2
                closestRoomba = position
                closestRoomba.z = 2.5
        return closestRoomba

    def run(self):
        self.test_follow()
        #self.test_explore()
        #self.test_follow_redirect()


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

        self.startSub = rospy.Subscriber('start_round', StartRound, self._on_start)
        self.roombaSub = rospy.Subscriber('visible_roombas', RoombaList, self._on_roombas)

    def _on_start(self, msg):
        if msg.start:
            self.round_start_time = msg.time
            #self.round_start_time = rospy.Time.now()
        self.has_started = msg.start

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
