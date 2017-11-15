"""
Heatmap.py
Written by Lydia Zuehsow (Oktober13) c. Fall 2017

This code contains simulation code that attempts to predict the location of Roombas over time.
There are two main types of simulators:
longSim- calculates long term frequency of square traversals, for a holistic view of which squares are most likely to have a Roomba in them at each moment.
        This accounts for collisions, and takes a longer time to calculate. It's really intended as an aid to overall strategy.
fiveSecSim- calculates less detailed short term estimates of Roomba location. Does not handle collisions. 
        This is intended for a realtime constantly-running Roomba position prediction.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import random
from collections import Counter

import rospy
import tf.transformations

from iarc_main.msg import Roomba, sys
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, Pose, Point, Quaternion

# from roomba_classes import TargetRoomba
# from scipy.misc import imread

import rospkg

rospack = rospkg.RosPack()
iarc_sim_path = rospack.get_path('iarc_sim_2d')
sys.path.append(np.os.path.join(iarc_sim_path, 'src'))

import config as cfg


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


def fiveSecSim(last_turn, last_noise, start_pose, end_time):
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
    # Generate 20 hypothetical Roombas' positions
    roomba_pos = [(0, 0)] * 20

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
                print cycle, roomba_pos[roomba]
                orient += random.randint(-(math.pi / 18), (math.pi / 18))
                # orient = orient + -math.pi / 9

                # Calculate dt
                dt = 5.0
                if remaining_duration < 5.0:
                    dt = remaining_duration

                roomba_pos[roomba] = locCalc(roomba_pos[roomba], dt, orient)
                remaining_duration -= dt

    # rospy.loginfo_throttle("Simulated positions: {}".format(roomba_pos))

    mean_pos = np.mean(roomba_pos, axis=0)
    mean_x = mean_pos[0]
    mean_y = mean_pos[1]
    mean_quaternion = tf.transformations.quaternion_from_euler(0, 0, mean_pos[2])
    msg = Pose(position=Point(mean_x, mean_y, 0), orientation=mean_quaternion)

    # TODO: Calculate covariance using eigenvectors of covariance matrix
    cov = [0] * 36
    cov[0], cov[7] = np.std(roomba_pos, axis=0)

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


def test_fiveSecSim():
    last_noise = rospy.Time(secs=0)
    last_turn = rospy.Time(secs=0)
    end_time = rospy.Time(secs=7)

    # Create a PoseWithCovariance
    header = rospy.Header(stamp=rospy.Time(secs=2))
    q = tf.transformations.quaternion_from_euler(0, 0, math.pi)
    msg = Pose(position=Point(0, 0, 0), orientation=Quaternion(q[0], q[1], q[2], q[3]))
    pose = PoseWithCovariance(pose=msg)
    start_pose = PoseWithCovarianceStamped(header=header, pose=pose)

    roomba_pos = fiveSecSim(last_turn, last_noise, start_pose, end_time)


def main():  # Needs to be cleaned up a bit
    # num_targets = 1
    # num_obstacles = 0
    # roomba_pos = longSim(num_targets,num_obstacles)

    test_fiveSecSim()

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
