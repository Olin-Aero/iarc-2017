#!/usr/bin/env python
import numpy as np
import config as cfg
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
            'front': False,
            'top': False
        }

        self.timers = {
            'reverse': 0,
            'noise': 0
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
                print("Turning clockwise")
                self.z_w = -cfg.ROOMBA_ANGULAR_SPEED
                self.heading -= amount
            else:
                print("Turning counterclockwise")
                self.z_w = cfg.ROOMBA_ANGULAR_SPEED
                self.heading += amount
            
            if self.turn_target < 0:
                # we have completed the turn, reset to forward motion
                print("turn completed")
                self.z_w = 0
                self.state = cfg.ROOMBA_STATE_FORWARD


class ObstacleRoomba(Roomba):
    '''
    Represents an obstacle roomba.
    '''

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
        if self.collisions['front']:
            self.collisions['front'] = False
        elif self.state == cfg.ROOMBA_STATE_FORWARD:
            self.pos[0] += cfg.ROOMBA_LINEAR_SPEED * np.cos(self.heading) * delta
            self.pos[1] += cfg.ROOMBA_LINEAR_SPEED * np.sin(self.heading) * delta

            # reorient so we tangent to a circle centered at the origin 
            ang = np.arctan2(10 - self.pos[1], 10 - self.pos[0])
            self.heading = ang + (cfg.PI / 2)
