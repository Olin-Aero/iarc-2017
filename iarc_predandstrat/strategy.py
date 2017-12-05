# from transitions import Machine
import numpy as np
import os
import rospkg
import sys
# from pygraphviz import *
import time
from enum import Enum
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Twist
from iarc_main.msg import Roomba, RoombaList
from std_msgs.msg import Float64

from PredictionEngine import PredictionEngine

rospack = rospkg.RosPack()
iarc_sim_path = rospack.get_path('iarc_sim_2d')
sys.path.append(os.path.join(iarc_sim_path, 'src'))

from config import XY_VEL, Z_VEL, LAND_IN_FRONT_DIST, ROOMBA_HEIGHT


class Action(Enum):
    """
    Action Enum for all possible actions.
    """
    TOPHIT = 1
    LANDINFRONT = 2
    TOPHIT2X = 3
    TOPHIT3X = 4
    WAIT = 5


class Drone(object):
    def __init__(self, C1=1, C2=1, C3=1, C4=1, C5=1, MIN_OBSTACLE_DISTANCE=1.5, FIELD_SIZE=20.0):
        self.tf = tf.TransformListener()
        self.predictor = PredictionEngine(tf_listener=self.tf)
        self.tag = 'drone'
        rospy.Subscriber('/seen_roombas', RoombaList, self.recordVisible)
        rospy.Subscriber('/drone/height', Float64, self.recordHeight)
        self.vel3d = Twist()
        self.visibleRoombas = []  # type: list[Roomba]
        self.visibleObstacles = []  # type: list[Roomba]

        self.C1 = C1
        self.C2 = C2
        self.C3 = C3
        self.C4 = C4
        self.C5 = C5
        self.MIN_OBSTACLE_DISTANCE = MIN_OBSTACLE_DISTANCE

        self.FIELD_SIZE = FIELD_SIZE

    def recordVisible(self, msg):
        """
        :param (RoombaList) msg:
        """
        self.visibleRoombas = [x for x in msg.data if x.type in (Roomba.RED, Roomba.GREEN)]
        self.visibleObstacles = [x for x in msg.data if x.type == Roomba.OBSTACLE]

    def recordHeight(self, msg):
        """
        :param (Float64) msg:
        :return:
        """
        self.height = msg.data

    def getPose(self):
        self.drone_pos, self.drone_heading = self.tf.lookupTransform(
            'map', '%s' % drone.tag, rospy.Time(0)
        )

    def actionTimeEstimate(self, target, action):
        """
        Based on the action entered, returns an estimate of how long the
        action will take to execute.
        :param action:
        :param target: Roomba
        :return:
        """
        self.getPose()
        t = 0 #Initialize time estimate

        print(action)
        if action == Action.TOPHIT:

            x_d = self.drone_pos[0] - target.visible_location.pose.pose.position.x
            y_d = self.drone_pos[1] - target.visible_location.pose.pose.position.y
            d_xy = np.sqrt(x_d ** 2 + y_d ** 2)
            t = d_xy/XY_VEL + 0*(self.height-ROOMBA_HEIGHT)/Z_VEL
        elif action == Action.LANDINFRONT:
            angle = self.orientationToHeading(target.visible_location.pose.pose.orientation)
            x_d = self.drone_pos[0] - (target.visible_location.pose.pose.position.x + LAND_IN_FRONT_DIST*np.cos(angle))
            y_d = self.drone_pos[1] - (target.visible_location.pose.pose.position.y + LAND_IN_FRONT_DIST*np.sin(angle))
            d_xy = np.sqrt(x_d ** 2 + y_d ** 2)
            t = d_xy/XY_VEL + 0*(self.height-ROOMBA_HEIGHT)/Z_VEL

        return t

    def orientationToHeading(self, orientation):
        """
        Converts a quaterion in the form of a pose orientation into a heading.
        :param orientation:
        :return:
        """
        res = [0, 0, 0, 0]
        res[0] = orientation.x
        res[1] = orientation.y
        res[2] = orientation.z
        res[3] = orientation.w
        return tf.transformations.euler_from_quaternion(res)[2]

    def goodnessScore(self):
        """
        Determines which Roomba we pick to lead to the goal.
        Higher score is better.

        Returns: [(Roomba, Score)]
        """

        def headingScore(roomba):
            # print(roomba.visible_location.pose.pose.orientation)
            heading = self.orientationToHeading(roomba.visible_location.pose.pose.orientation)


            return np.sin(heading)

        def positionScore(roomba):
            return roomba.visible_location.pose.pose.position.y

        def distanceFromObstaclesScore(roomba, obstacles):
            """
            (-infinity, 0)
            """

            score = 0
            for obstacle in obstacles:
                x = roomba.x - obstacle.x
                y = roomba.y - obstacle.y
                dist = np.sqrt(x ** 2 + y ** 2)

                if dist < MIN_OBSTACLE_DISTANCE:
                    return -math.inf

                score -= 1 / dist ** 2

            return score

        def stateQualtityScore(roomba):
            """
            How precisely we know the Roombas' state.
            Compare position accuracy to view radius to know if it's possible
            to see the given roomba when drone arrives.
            """
            return 0

        def futureGoodnessScore(roomba):
            return 0

        result = []

        for i in xrange(0, len(self.visibleRoombas)):
            roomba = self.visibleRoombas[i]

            score = self.C1 * headingScore(roomba) + \
                    self.C2 * positionScore(roomba) + \
                    self.C3 * distanceFromObstaclesScore(roomba, self.visibleObstacles) + \
                    self.C4 * stateQualtityScore(roomba) + \
                    self.C5 * futureGoodnessScore(roomba)
            result.append((roomba, score))

        return result

    def targetSelect(self, roombaScore):
        # print(roombaScore)
        # print(roombaScore != [])
        if roombaScore != []:
            # return [0,Drone()]
            # print(max(roombaScore, key=lambda x: x[1]))
            return max(roombaScore, key=lambda x: x[1])
        else:
            res = Drone()
            # print("Garb")
            res.tag = ''
            return [res, 0]

    def chooseAction(self, target):
        """
        Determined what the best action to take is for the given target
        :param (Roomba) target: The Roomba we intend to interact with
        :return: The best action to take
        """
        # The maximum angle the target may deviate from North/South for us to want to turn it 45 degrees.
        ANGLE_WINDOW = np.deg2rad(30)
        TURN_TIME = rospy.Duration.from_sec(4.0)

        desired_angle = 0  # Want target going north

        # Step 1: check for a win condition
        roombaWillWin = False
        for dt in range(0, 20, 3):
            prediction = self.predictor.predict_future_position(target, rospy.Time.now() + rospy.Duration.from_sec(dt))
            if prediction.pose.pose.position.y > self.FIELD_SIZE / 2:
                roombaWillWin = True
                break

        if roombaWillWin:
            return 'follow'

        # Step 2: check for 45 degree turns

        current_angle = tf.transformations.euler_from_quaternion(target.visible_location.pose.pose.orientation)[2]

        angle_delta = self.subtract_angles(desired_angle, current_angle)
        if ANGLE_WINDOW < abs(angle_delta) < np.pi - ANGLE_WINDOW:
            # The roomba is going mostly perpendicular
            return '45'

        # Step 3: check for 180 degree turns
        expected_time = rospy.Time.now() + TURN_TIME
        expected_pos = self.predictor.predict_future_position(target, expected_time)
        cycle_phase = (expected_time - target.last_turn).to_sec() % 20

        expected_angle = tf.transformations.euler_from_quaternion(expected_pos.pose.pose.orientation)[2]
        angle_delta = self.subtract_angles(desired_angle, expected_angle)

        if abs(angle_delta) > np.pi / 2 and cycle_phase <= 18:
            # The roomba is going the wrong way and not about to turn
            return '180'

        return 'follow'

    @staticmethod
    def subtract_angles(a, b):
        res = a - b

        while res <= -np.pi:
            res += 2 * np.pi
        while res > np.pi:
            res -= 2 * np.pi

        return res

rospy.init_node('strategy')

drone = Drone()
while not rospy.is_shutdown():
    time.sleep(.1)
    print('_' * 80)
    # print(drone.goodnessScore())
    if drone.goodnessScore() != []:
        for roomba, score in drone.goodnessScore():
            # print(score)
            drone.actionTimeEstimate(roomba, Action.LANDINFRONT)
            print('score: %f roomba: %s' % (score, roomba.frame_id))

        bestRoomba, bestRoombaScore = drone.targetSelect(drone.goodnessScore())
        print('Best Score: %f Best Roomba: %s' % (bestRoombaScore, bestRoomba.frame_id))


states = [
    'init',
    'search',
    'follow',
    'waitForValidRoomba',  # If a roomba is not availible to contact
    'avoidObstacle',
    'landInFront',
    'pushButton',
    'waitForGoodRoomba']

transitions = [
    {'trigger': 'start', 'source': 'init', 'dest': 'search'},
    {'trigger': 'goodRoombaFound', 'source': 'search', 'dest': 'follow'},
    {'trigger': 'timeInvalid', 'source': 'follow', 'dest': 'waitForValidRoomba'},
    # timeInvalid signals that it's not the right time to follow the roomba.

    # Null actions result in returning to search algorithm
    {'trigger': 'null', 'source': 'follow', 'dest': 'search'},
    {'trigger': 'null', 'source': 'waitForValidRoomba', 'dest': 'search'},
    {'trigger': 'null', 'source': 'waitForGoodRoomba', 'dest': 'search'},
    {'trigger': 'null', 'source': 'pushButton', 'dest': 'search'},
    {'trigger': 'null', 'source': 'landInFront', 'dest': 'search'},

    # Action triggers
    {'trigger': '180Needed', 'source': 'follow', 'dest': 'landInFront'},
    {'trigger': '45Needed', 'source': 'follow', 'dest': 'pushButton'},
    {'trigger': 'timeElapsedNeeded', 'source': 'follow', 'dest': 'waitForGoodRoomba'},

    {'trigger': 'actionCompleted', 'source': 'landInFront', 'dest': 'follow'},
    {'trigger': 'actionCompleted', 'source': 'pushButton', 'dest': 'follow'},
    {'trigger': 'actionCompleted', 'source': 'waitForGoodRoomba', 'dest': 'follow'},

    # Obstacle triggers
    {'trigger': 'obstacleInPath', 'source': 'search', 'dest': 'avoidObstacle'},
    {'trigger': 'obstacleInPath', 'source': 'follow', 'dest': 'avoidObstacle'},
    {'trigger': 'obstacleInPath', 'source': 'waitForValidRoomba', 'dest': 'avoidObstacle'},
]

machine = Machine(model=drone, states=states, transitions=transitions, initial='init')

drone.get_graph().draw('state_diagram.png', prog='dot')

drone.start()
print(drone.state)

print(cfg.ROOMBA_HEIGHT)
drone.goodRoombaFound()

print(drone.state)
