"""
Written by Khang c. Spring 2018

This code calculates less detailed short term estimates
of Roomba location. Does not handle collisions.
This is intended for a real-time constantly-running
Roomba position prediction.
"""


import math
import random

import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, Pose, Point, Quaternion


def fiveSecSim(last_turn, last_noise, start_pose, end_time):
    """
    This simulation is for short term rapid location estimation of a Roomba that hasn't been seen for a while.
    20 hypothetical "Imaginary Roombas" trace out paths it could have taken,
    and the centroid of the final scatter determines the hypothesized final position of the actual Roomba.
    times_since = Array. Time since the Roomba was last seen.

    :arg (rospy.Time) last_turn: The last timestamp at which the Roomba turned (every 20 seconds)
    :arg (rospy.Time) last_noise: The last timestamp at which the Roomba had noise
    :arg (PoseWithCovarianceStamped) start_pose: The start pose of the Roomba
    :arg (rospy.Time) end_time: When we want to simulate until
    :returns prediction: The estimated final position
    :rtype: PoseWithCovarianceStamped
    """
    # Generate 20 hypothetical Roombas' positions
    roomba_pos = [(0, 0)] * 20

    # The latest time at which the Roomba's location is known
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

    # TODO: Calculate covariance using eigenvectors of covariance matrix
    cov = [0] * 36
    cov[0], cov[7], junk = np.std(roomba_pos, axis=0)

    # print np.cov([roomba_pos[0],roomba_pos[1]],mean_pos)

    pad = np.zeros(20)
    yaw = tf.transformations.quaternion_from_euler(pad,pad,np.array(roomba_pos)[:,2])
    print yaw
    XYZRPY = [np.array(roomba_pos)[:,0],np.array(roomba_pos)[:,1],pad,pad,pad,np.array(roomba_pos)[:,2]]

    # XYcovariance = np.cov(np.array(roomba_pos)[:,0],np.array(roomba_pos)[:,1])
    # print "XY cov: ", covariance
    # eigvals, eigvecs = np.linalg.eig(covariance)
    # print "Eigenvalues", eigvals
    # print "Eigenvectors", eigvecs


    # print "XY: ", np.array(roomba_pos)[:,0],np.array(roomba_pos)[:,1]
    # print "Orientation: ", [mean_quaternion[0],mean_quaternion[1],mean_quaternion[2]]
    # print "Orientation cov:", np.cov([mean_quaternion[0],mean_quaternion[1],mean_quaternion[2]])


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