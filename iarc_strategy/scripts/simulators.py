"""
Simulators.py
Written by Lydia Zuehsow (Oktober13) c. Fall 2017

This code contains simulation code that attempts to predict the location of Roombas over time.
There are two main types of simulators:
longSim- calculates long term frequency heatmap of square traversals, for a holistic view of which squares are most likely to have a Roomba in them at each moment.
        This accounts for collisions, and takes a longer time to calculate. It's really intended as an aid to overall strategy.
futureSim- calculates less detailed short term estimates of Roomba location, with covariance data. Returns Pose Stamped with Covariance. Does not handle Roomba collisions. 
        This is intended for a realtime constantly-running Roomba position prediction.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import random
import os
from collections import Counter

import rospy
import tf.transformations

import enum

from iarc_main.msg import Roomba, sys
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, Pose, Point, Quaternion

# from roomba_classes import TargetRoomba
# from scipy.misc import imread

import rospkg

rospack = rospkg.RosPack()
iarc_sim_path = rospack.get_path('iarc_sim_2d')
# sys.path.append(np.os.path.join(iarc_sim_path, 'src'))
sys.path.append(os.path.join(iarc_sim_path, 'src'))

import config as cfg


class Action(enum.Enum):
    """
    Action Enum for all possible events that can occur.

    """
    COLLISION = 1
    LANDINFRONT = 2
    TOPHIT = 3
    TOPHIT2X = 4
    TOPHIT3X = 5
    NOISE = 6
    TURN = 7

class SimRoomba(object):
    def __init__(self, targets=0, obstacles=0):
        self.t = TargetRoomba([0, 0], (math.pi / 2.000), 'Bob')  # Creates TargetRoomba object (See robots.py)
        self.t.start()
        # t.prev_pos = []
        self.t.curr_pos = (0, 0)

    def updatePos(self, time_passed, time_since=1):
        try:
            self.t.update(time_since, time_passed * 1000)
            self.t.curr_pos = (int(math.floor(self.t.pos[0])), int(math.floor(self.t.pos[1])))
            print time_passed, (math.floor(self.t.pos[0]), math.floor(self.t.pos[1])), self.t.curr_pos
            # self.t.roomba_pos.append((int(math.floor(t.pos[0])),int(math.floor(t.pos[1]))))
            # self.t.curr_pos = (x,y)
            # self.t.prev_pos.append(t.curr_pos)
            return True
        except:
            return False

    def getPos(self):
        return self.t.curr_pos


# class LocationSim(object):
#     def __init__(sekd)

def plotStuff(loc_data):
    """
    This functions plots a long term probabilistic heatmap of the squares most likely to be traversed by Roombas, given the past positions of all Roombas.

    :arg (dict) loc_data: Dictionary of each Roomba and all of the squares traversed by its simulated future self.
    """
    # cluster = {{(4, -2): 8, (0, -2): 8, (-3, -1): 6} # Useful fake data for testing things
    # print loc_data.values() 

    fig = plt.figure()

    background = imread("grid.jpg")

    for (coords, numtimes) in loc_data.items():
        (x, y) = coords
        color = (float(numtimes) / len(loc_data.items())) * 255.
        # print color

        # Currently trying to debug this- Color is displaying as all red, but should be a variety of shades based on that set of coordinates frequency of recurrence.

        plt.scatter(x, y, c=color, cmap='gist_rainbow')
    plt.imshow(background, zorder=0, extent=[-10.0, 10.0, -10.0, 10.0])

    plt.show()

def populateQueue(passed_actions, passed_turns, passed_noise):
    """
    This function populates a queue of yaw transformations that the roomba must go through, along with the time to when they will occur.

    :arg (list) passed_actions: List of actions to be performed.
    :arg (list) passed_turns: List of tuples (num_turns, time_to_turn)
    :arg (list) passed_noise: List of tuples (num_noise, time_to_noise)
    :var (int) num_turns: Number of turns the Roomba must make before the simulation ends.
    :var (float) time_to_turn: Time to the first turn that the Roomba must make. Based on estimate of time passed since last observed turn for a Roomba.
    :var (int) num_noise: Number of "noise" yaw adjustments the Roomba must make before the simulation ends.
    :var (float) time_to_noise: Time to the first noise that the Roomba will experience. Based on estimate of time passed since last observed noise for a Roomba.
    :returns event_queue: The queue of events (yaw transformations on the Roomba's orientation), sorted by time to event.
    """

    (num_turns, time_to_turn) = passed_turns
    (num_noise, time_to_noise) = passed_noise

    action_list = passed_actions
    turns_list = []
    noise_list = []
    event_queue = []

    for num in range(num_turns):
        turns_list.append((Action.TURN, time_to_turn+(20.0*num)))
    for num in range(num_noise):
        noise_list.append((Action.NOISE, time_to_noise+(5.0*num)))

    if action_list is not None:
        event_queue = event_queue + action_list
    if turns_list is not None:
        event_queue = event_queue + turns_list
    if noise_list is not None:
        event_queue = event_queue + noise_list
    event_queue = sorted(event_queue, key=lambda x: x[1])
    return event_queue

def performEvent(passed_orient, passed_event):
    """
    This function identifies event type and implements the appropriate transforms on Roomba yaw orientation.

    :arg (Action) passed_event: The current event (Action) being carried out.
    :arg (float) passed_orient: The current orientation (yaw, in radians) of the Roomba.
    :var (float) orient: The final orientation (yaw, in radians) of the Roomba.
    :var (float) event_length: Length (seconds) of time the event takes to complete.
    :returns (orient, event_length): The new orientation and the estimated length the event takes to complete
    """

    assert isinstance(passed_event, Action), \
    "%r is not a valid event." % (passed_event)

    if passed_event is Action.COLLISION: # Turn 180 deg. in 2 second.
        orient = passed_orient + math.pi
        event_length = 2.0
    elif passed_event is Action.TOPHIT: # Turn 90 deg. in 1 second.
        orient = passed_orient + (math.pi / 2.0000)
        event_length = 1.0
    elif passed_event is Action.TOPHIT2X: # Turn 180 deg. in 2 seconds.
        orient = passed_orient + (math.pi)
        event_length = 2.0
    elif passed_event is Action.TOPHIT3X: # Turn 270 deg. in 3 seconds.
        orient = passed_orient + (math.pi * 1.5000)
        event_length = 3.0
    elif passed_event is Action.TURN: # Turn 180 deg. in 2 seconds.
        orient = passed_orient + (math.pi)
        event_length = 2.0
    elif passed_event is Action.NOISE: # Turn random amount +/- 10 deg.
        orient = passed_orient + random.uniform(-(math.pi / 18), (math.pi / 18))
        event_length = 0.3
    else:
        orient = passed_orient
        event_length = 0.0
    # assert orient is not None
    return (orient, event_length)

def futureSim(passed_roomba_state, passed_actions, start_pose):
    """
    This simulation is for short term rapid location estimation of a Roomba that hasn't been seen for a while.
    20 hypothetical "Imaginary Roombas" trace out paths it could have taken,
    and the centroid of the final scatter determines the hypothesized final position of the actual Roomba.
    times_since = Array. Time since the Roomba was last seen.

    :arg (rospy.Time) last_turn: The last time the Roomba turned every 20 seconds
    :arg (rospy.Time) last_noise: The last time the Roomba had noise
    :arg (PoseWithCovarianceStamped) start_pose: The start pose of the Roomba
    :arg (rospy.Time) end_time: When we want to simulate until
    :returns prediction: The estimated final position
    :rtype: PoseWithCovarianceStamped
    """
    (last_turn, last_noise, end_time) = passed_roomba_state

    roomba_pos = [(0, 0)] * 20 # Generate 20 hypothetical Roombas' positions

    known_time = start_pose.header.stamp

    sim_duration = (end_time - known_time).to_sec()

    start_pos = start_pose.pose.pose.position

    quaternion = start_pose.pose.pose.orientation
    quaternion_arr = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    start_orient = tf.transformations.euler_from_quaternion(quaternion_arr)[2]

    # Check time remaining until Roomba experiences noise
    time_to_noise = 5 - (known_time - last_noise).to_sec() % 5
    # Check number of times Roombas should go through "noise"
    noise_cycles = int(math.ceil((sim_duration - time_to_noise) / 5.0))

    # Check time remaining until Roomba turns
    time_to_turn = 20 - (known_time - last_turn).to_sec() % 20
    # Check number of times Roombas should go through "turn cycles"
    turn_cycles = int(math.ceil((sim_duration - time_to_turn) / 20.0))

    event_queue = populateQueue(passed_actions,(turn_cycles,time_to_turn),(noise_cycles,time_to_noise))

    # For one of 20 hypothetical roombas...
    for roomba in range(0, len(roomba_pos)):
        print "Roomba %i" % roomba
        orient = start_orient
        prev_time = 0.0
        roomba_pos[roomba] = start_pos.x, start_pos.y, orient

        for item in event_queue:
            (event, time_to_event) = item

            assert (time_to_event >= prev_time) or ((event is Action.TURN) or (event is Action.NOISE) or (event is Action.COLLISION)), \
            "Event time error. %r occurs too rapidly after previous event." % (event)

            if time_to_event - prev_time >= 0: roomba_pos[roomba] = locCalc(roomba_pos[roomba], time_to_event - prev_time, orient)
            (orient, event_length) = performEvent(orient, event)
            roomba_pos[roomba] = locCalc(roomba_pos[roomba], 0.0, orient)
            prev_time = time_to_event + event_length

    # rospy.loginfo_throttle("Simulated positions: {}".format(roomba_pos))

    mean_pos = np.mean(roomba_pos, axis=0)
    # print mean_pos
    mean_x = mean_pos[0]
    mean_y = mean_pos[1]
    mean_quaternion = tf.transformations.quaternion_from_euler(0, 0, mean_pos[2])
    msg = Pose(position=Point(mean_x, mean_y, 0), orientation=Quaternion(mean_quaternion[0], mean_quaternion[1], mean_quaternion[2], mean_quaternion[3]))

    # TODO: Calculate variance using eigenvectors of covariance matrix

    pad = np.zeros(20)
    XYZRPY = [np.array(roomba_pos)[:,0],np.array(roomba_pos)[:,1],pad,pad,pad,np.array(roomba_pos)[:,2]]

    cov = np.cov(XYZRPY)

    print cov

    # eigvals, eigvecs = np.linalg.eig(covariance)
    # print "Eigenvalues", eigvals
    # print "Eigenvectors", eigvecs

    return PoseWithCovarianceStamped(
        header=start_pose.header,
        pose=PoseWithCovariance(
            pose=msg,
            covariance=cov
        )
    )


def locCalc(start_loc, dt, orient, vel=0.33):
    """
    current position = previous position + velocity * time
    Helper function. Calculates new position given previous location, time spent moving, orientation, and velocity.
    Units are in meters.
    :arg (tuple) start_loc: x, y, orientation of the roomba
    :arg (Float) dt: Moving duration
    :arg (Float) orient: Orientation of the Roomba in radian
    :arg (Float) vel: roomba's velocity
    :returns: x, y, orientation: The roomba's position after dt seconds
    :rtype: Float, Float, Float
    """
    past_x, past_y, past_orient = start_loc
    curr_x = past_x + dt * vel * math.cos(orient)
    curr_y = past_y + dt * vel * math.sin(orient)

    return curr_x, curr_y, orient


def longSim(passed_num_targets, passed_num_obstacles):
    """
    This simulation is for long term frequency distribution type calculations, to determine the most commonly traversed squares for all Roombas.
    It's mapped to the rainbow colormap, so reds are least common, and violets are most common intersections.
    """
    roomba_pos = []

    bob = SimRoomba(passed_num_targets, passed_num_obstacles)

    for timer in range(0, 100):
        if bob.updatePos(timer):
            roomba_pos.append(bob.getPos())
        else:
            print("Long term simulation failed")
            return roomba_pos
    return roomba_pos

def chooseSimType():
    pass


def test_futureSim(passed_roomba_state,passed_actions):
    # Create a PoseWithCovariance
    header = rospy.Header(stamp=rospy.Time(secs=2))
    q = tf.transformations.quaternion_from_euler(0, 0, math.pi)
    msg = Pose(position=Point(0, 0, 0), orientation=Quaternion(q[0], q[1], q[2], q[3]))
    pose = PoseWithCovariance(pose=msg)
    start_pose = PoseWithCovarianceStamped(header=header, pose=pose)

    roomba_pos = futureSim(passed_roomba_state, passed_actions, start_pose)
    return


def main():
    # num_targets = 1
    # num_obstacles = 0
    # roomba_pos = longSim(num_targets,num_obstacles)


    last_noise = rospy.Time(secs=0)
    last_turn = rospy.Time(secs=0)
    sim_end_time = rospy.Time(secs=50)

    roomba_state = (last_noise,last_turn,sim_end_time)
    action_list = [(Action.LANDINFRONT,2.0), (Action.COLLISION,2.0)]
    test_futureSim(roomba_state, action_list)


    # # Used for Long Term Simulator
    # print roomba_pos
    # tc = Counter(roomba_pos)  # Dictionary of Roomba positions.
    # print(tc)

    # heatmap = [[0 for col in range(-10,10)] for row in range(-10,10)] # Not currently used- an array with frequency values for roomba traversal in every square.
    # for key in tc:
    #     heatmap[key[0]][key[1]] = heatmap[key[0]][key[1]] + 1
    # # print heatmap

    # plotStuff(tc)


if __name__ == '__main__':
    main()
