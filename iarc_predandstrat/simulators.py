"""
Heatmap.py
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

from enum import Enum

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


class Action(Enum):
    """
    Action Enum for all possible actions.
    """
    COLLISION = 1
    TOPHIT = 2
    TOPHIT2X = 3
    TOPHIT3X = 4

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

    def updateOrient(input_action):
        if Action.COLLISION: # Turn 180 deg.
            pass
        elif Action.TOPHIT:
            pass
        elif Action.TOPHIT2X:
            pass
        elif Action.TOPHIT3X:
            pass
        else:
            pass

    def getPos(self):
        return self.t.curr_pos


# class LocationSim(object):
#     def __init__(sekd)

def plotStuff(loc_data):
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

    # TODO: consider a separate timer for noise, roomba.last_turn is about 180 reversals
    # Check time remaining until Roomba experiences noise
    time_to_noise = 5 - (known_time - last_noise).to_sec() % 5
    # Check number of times Roombas should go through "noise"
    noise_cycles = int(math.ceil((sim_duration - time_to_noise) / 5.0))

    # Check time remaining until Roomba turns
    time_to_turn = 20 - (known_time - last_turn).to_sec() % 20
    # Check number of times Roombas should go through "turn cycles"
    turn_cycles = int(math.ceil((sim_duration - time_to_turn) / 20.0))

    if passed_actions:
        passed_actions.reverse()
        current_action = passed_actions.pop()
        (action_type, time_to_action) = current_action
    else:
        time_to_action = None

    # For one of 20 hypothetical roombas...
    for roomba in range(0, len(roomba_pos)):
        print "Roomba %i" % roomba
        remaining_duration = sim_duration
        orient = start_orient
        roomba_pos[roomba] = start_pos.x, start_pos.y, orient

        if noise_cycles <= 0:
            # If sim_duration < time_to_noise (first noise)
            roomba_pos[roomba] = locCalc(roomba_pos[roomba], sim_duration, orient)
            remaining_duration -= sim_duration
        else:
            # The first simulation period is time_to_noise seconds long
            dt = time_to_noise
            roomba_pos[roomba] = locCalc(roomba_pos[roomba], dt, orient)
            remaining_duration -= dt

            for cycle in range(1, noise_cycles + 1):
                orient += random.uniform(-(math.pi / 18), (math.pi / 18))
                # orient = orient + -math.pi / 9

                # Calculate dt
                dt = 5.0
                if remaining_duration < 5.0:
                    dt = remaining_duration

                roomba_pos[roomba] = locCalc(roomba_pos[roomba], dt, orient)
                remaining_duration -= dt
                # print roomba_pos[roomba] #X, y, orientation

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


def main():  # Needs to be cleaned up a bit
    # num_targets = 1
    # num_obstacles = 0
    # roomba_pos = longSim(num_targets,num_obstacles)


    last_noise = rospy.Time(secs=0)
    last_turn = rospy.Time(secs=0)
    sim_end_time = rospy.Time(secs=10)

    roomba_state = (last_noise,last_turn,sim_end_time)
    # action_list = [(Action.LANDINFRONT,2)]
    action_list = []
    test_futureSim(roomba_state, action_list)

    # print roomba_pos
    # tc = Counter(roomba_pos)  # Dictionary of Roomba positions
    # print(tc)

    # heatmap = [[0 for col in range(-10,10)] for row in range(-10,10)] # Not currently used- an array with frequency values for roomba traversal in every square.
    # for key in tc:
    #     heatmap[key[0]][key[1]] = heatmap[key[0]][key[1]] + 1
    # # print heatmap

    # plotStuff(tc)


if __name__ == '__main__':
    main()
