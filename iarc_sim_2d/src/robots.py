#!/usr/bin/env python
import numpy as np

import config as cfg
import rospy
import tf
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from random import choice
'''
roomba.py

Contains two classes to represent target and obstacle roombas.

Behavior:

# Target Roombas
- Every 20 seconds, 180 deg turn clockwise
- Every 5 seconds, +-20 deg turn
- Front collision -> 180 deg turn clockwise
- Top collision -> 45 deg turn clockwise

# Obstacle Roombas
- Move in a circular motion around the origin
- Front collision -> idle until unobstructed
'''


class Roomba(object):
    '''
    Represents a generic roomba.
    (No update function)
    '''

    def __init__(self, pos, heading, tag=''):
        '''
        Initialize a roomba object with a given position and heading.
        By default, the roomba starts in STATE_IDLE.

        pos - [x,y] in meters
        heading - angle in radians (0 is +x and pi/2 is +y)
        [tag] - an optional identification element
        '''
        self.pos = pos
        self.heading = heading
        self.tag = tag
        self.x_vel = 0
        self.z_w = 0

        self.collisions = {
            'front' : False,
            'top' : False
        }

        self.timers = {
            'reverse': 0,
            'noise': 0,
            'stopped': 0
        }

        self.state = cfg.ROOMBA_STATE_IDLE

        # amount we need to turn
        self.turn_target = 0
        self.turn_clockwise = False

    def start(self):
        '''
        Sets the roomba to STATE_FORWARD
        '''
        self.state = cfg.ROOMBA_STATE_FORWARD

    def stop(self):
        '''
        Sets the roomba to STATE_IDLE

        Note: if the roomba was mid-turn during this call, it will
        not resume after a restart.
        '''
        self.state = cfg.ROOMBA_STATE_IDLE

    def update(self, delta, elapsed):
        '''
        Perform an update step (unimplemented)
        '''
        pass


class TargetRoomba(Roomba):
    '''
    Represents a target roomba.
    '''

    def update(self, delta, elapsed):
        '''
        Perform an update step.

        delta - change in time since last update (seconds)
        elapsed - total time elapsed since start (milliseconds)
        '''
        # check for collisions
        if self.collisions['front']:
            self.collisions['front'] = False

            if self.state == cfg.ROOMBA_STATE_FORWARD:
                self.state = cfg.ROOMBA_STATE_TURNING
                self.turn_target = np.pi
                self.turn_clockwise = True

        if self.collisions['top']:
            self.collisions['top'] = False
            if self.state == cfg.ROOMBA_STATE_FORWARD:
                self.state = cfg.ROOMBA_STATE_TURNING
                self.turn_target = np.pi/4
                self.turn_clockwise = True

        # update linear or angular motion
        if self.state == cfg.ROOMBA_STATE_FORWARD:
            self.pos[0] += cfg.ROOMBA_LINEAR_SPEED * np.cos(self.heading) * delta
            self.pos[1] += cfg.ROOMBA_LINEAR_SPEED * np.sin(self.heading) * delta
            self.x_vel = cfg.ROOMBA_LINEAR_SPEED
            # check full turn period
            if elapsed - self.timers['reverse'] > cfg.ROOMBA_REVERSE_PERIOD:
                self.timers['reverse'] = elapsed
                self.state = cfg.ROOMBA_STATE_TURNING
                self.turn_target = np.pi
                self.turn_clockwise = True
            # check random noise period
            elif elapsed - self.timers['noise'] > cfg.ROOMBA_HEADING_NOISE_PERIOD:
                self.timers['noise'] = elapsed
                self.state = cfg.ROOMBA_STATE_TURNING
                self.turn_target = np.random.rand() * cfg.ROOMBA_HEADING_NOISE_MAX
                self.turn_clockwise = (np.random.rand() > 0.5)

        elif self.state == cfg.ROOMBA_STATE_TURNING:
            amount = cfg.ROOMBA_ANGULAR_SPEED * delta
            self.x_vel = 0
            self.turn_target -= amount

            if self.turn_clockwise:
                # print("Turning clockwise")
                self.z_w = -cfg.ROOMBA_ANGULAR_SPEED
                self.heading -= amount
            else:
                # print("Turning counterclockwise")
                self.z_w = cfg.ROOMBA_ANGULAR_SPEED
                self.heading += amount

            if self.turn_target < 0:
                # we have completed the turn, reset to forward motion
                # print("turn completed")
                self.z_w = 0
                self.state = cfg.ROOMBA_STATE_FORWARD
        elif self.state == cfg.ROOMBA_STATE_OOB:
            self.x_vel = 0
            self.z_w = 0

    def collision(self, self_pos, self_heading, other_pos, other_heading, self_radius=cfg.ROOMBA_RADIUS, other_radius=cfg.ROOMBA_RADIUS):
        h_i = tf.transformations.euler_from_quaternion(self_heading)[-1] # yaw
        u_i = [np.cos(h_i), np.sin(h_i)]

        dy = other_pos[1] - self_pos[1]
        dx = other_pos[0] - self_pos[0]

        d = np.sqrt(dx**2 + dy**2)
        if d < self_radius + other_radius and np.dot(u_i, [dx,dy]) > 0:
            self.collisions['front'] = True

    def bounds(self, self_pos):
        if abs(self_pos[0]) > (cfg.BOUND/2.0) or abs(self_pos[1]) > (cfg.BOUND/2.0):
            self.state = cfg.ROOMBA_STATE_OOB


class ObstacleRoomba(Roomba):
    '''
    Represents an obstacle roomba.
    '''
    def gen_pole(self):
        self.pole_height = cfg.getObstacleHeight()

    def update(self, delta, elapsed):
        '''
        Perform an update step.

        Note: in order to acheive circular motion, the heading is
        updated each step to be perpendicular to a vector from
        the center to the roomba. For small deltas, this should
        provide realistic behavior.

        delta - change in time since last update (seconds)
        elapsed - total time elapsed since start (milliseconds)
        '''
        # reorient so we tangent to a circle centered at the origin
        #ang = np.arctan2(10 - self.pos[1], 10 - self.pos[0])
        #self.heading = ang
        # self.x_vel = cfg.ROOMBA_LINEAR_SPEED

        direction = [-1, 1]

        if self.collisions['front']:
            self.collisions['front'] = False
            self.state = cfg.ROOMBA_STATE_IDLE
            self.timers['stopped'] = elapsed

        if elapsed - self.timers['stopped'] > cfg.ROOMBA_OBSTACLE_STOP_PERIOD:
            self.timers['stopped'] = elapsed
            self.state = cfg.ROOMBA_STATE_FORWARD

        if self.state == cfg.ROOMBA_STATE_IDLE:
            self.x_vel = 0
            self.z_w = 0

        if self.state == cfg.ROOMBA_STATE_FORWARD:
            self.x_vel = cfg.ROOMBA_LINEAR_SPEED
            self.z_w = self.x_vel / cfg.ROOMBA_OBSTACLE_TURN_RADIUS

            if elapsed - self.timers['noise'] > cfg.ROOMBA_OBSTACLE_NOISE_PERIOD:
                self.timers['noise'] = elapsed
                self.z_w += (choice(direction)) * cfg.ROOMBA_OBSTACLE_NOISE_MAX

        if self.state == cfg.ROOMBA_STATE_OOB:
            self.x_vel = 0
            self.z_w = 0

        #elif self.collisions['top']:
        #   self.collisions['top'] = False
        #   self.state = cfg.ROOMBA_STATE_IDLE
        #   self.timers['stopped'] = elapsed

        #    if elapsed - self.timers['stopped'] > cfg.ROOMBA_OBSTACLE_STOP_PERIOD:
        #       self.timers['stopped'] = elapsed
        #       self.state = cfg.ROOMBA_STATE_FORWARD

        # if self.state == cfg.ROOMBA_STATE_FORWARD:
        self.pos[0] += cfg.ROOMBA_LINEAR_SPEED * np.cos(self.heading) * delta
        self.pos[1] += cfg.ROOMBA_LINEAR_SPEED * np.sin(self.heading) * delta


    def collision(self, self_pos, self_heading, other_pos, other_heading, self_radius=cfg.ROOMBA_RADIUS, other_radius=cfg.ROOMBA_RADIUS):
        h_i = tf.transformations.euler_from_quaternion(self_heading)[-1] # yaw
        u_i = [np.cos(h_i), np.sin(h_i)]

        dy = other_pos[1] - self_pos[1]
        dx = other_pos[0] - self_pos[0]

        d = np.sqrt(dx**2 + dy**2)
        if d < self_radius + other_radius and np.dot(u_i, [dx,dy]) > 0:
            #print("Pole height is %f" %(cfg.ROOMBA_HEIGHT+self.pole_height))
            self.collisions['front'] = True

    def bounds(self, self_pos):
        if abs(self_pos[0]) > (cfg.BOUND/2.0) or abs(self_pos[1]) > (cfg.BOUND/2.0):
            self.state = cfg.ROOMBA_STATE_OOB

class Drone(object):
    """
    Represents the drone in the simulation
    """

    def __init__(self, pos2d, heading, tag = ''):
        """
        Initialize the Drone object where:
        pos3d is a vector [x,y,z]
        vel3d is a vector [x',y',z']
        heading is an angle in radians (0 is +x and pi/2 is +y)
        """
        initial_height = 0 #Make this an arugment at some point
        self.vel3d = Twist()

        self.pos3d = [pos2d[0], pos2d[1], initial_height]
        self.heading = heading
        self.tag = tag
        self.visible_roombas = []

        self.obsticle_bump_count = 0

        # makes it so that drone has to distance itself before it can collide again
        self.can_collide = True
        self.collision_center = []

        self.OutOfBounds = False
        self.TimeOOB = 0

    def limitSpeed(self, speedLimit):
        currentSpeed = np.linalg.norm(self.vel3d)
        if currentSpeed > speedLimit:
            self.vel3d = self.vel3d * speedLimit/currentSpeed

    def collision(self, self_pos, self_heading, other_pos, other_heading, self_radius=cfg.DRONE_RADIUS, other_radius=cfg.OBSTACLE_POLE_RADIUS):

        dy = other_pos[1] - self_pos[1]
        dx = other_pos[0] - self_pos[0]

        d = np.sqrt(dx**2 + dy**2)

        if self.can_collide == True:
            if d < self_radius + other_radius:
                print("Collision with obsticle roomba pole")
                self.obsticle_bump_count += 1

                self.collision_center = [other_pos[0], other_pos[1]]
                self.can_collide = False

        if self.can_collide == False:
            if np.sqrt((self.collision_center[0] - self.pos3d[0]) ** 2 + (self.collision_center[1] - self.pos3d[1]) ** 2) > self_radius + other_radius + 0.35:
                self.can_collide = True
                self.collision_center = []

    def record_vel(self, data):
        self.vel3d = data

    def update(self, delta, elapsed):
        z_vel = self.vel3d.linear.z
        self.pos3d[2] += z_vel * delta
        if self.pos3d[2] < 0:
            self.pos3d[2] = 0
        self.heightPublisher.publish(self.pos3d[2])

    def get_visible_roombas(self, roomba_array, pos_array):
        visible_roombas = []
        index_list = []
        for i in xrange(0, len(pos_array)):

            dy = pos_array[i][1] - self.pos3d[1]
            dx = pos_array[i][0] - self.pos3d[0]

            d = np.sqrt(dx**2 + dy**2)

            if d < np.tan(cfg.BOTTOM_CAMERA_FOV/2.0)*self.pos3d[2]:
                visible_roombas = np.append(visible_roombas, roomba_array[i])
                index_list.append(i)
        self.visible_roombas = visible_roombas
        self.index_list = index_list
