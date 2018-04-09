'''
config.py

Contains a bunch of modifiable constants organized by
module type.
'''
import re
import numpy as np

_const_rgx = re.compile('(([A-Z_][A-Z0-9_]*)|(__.*__))$')

def load(module):
    '''
    Copies all module-level constants from `module`
    into this module.

    Identifiers must match _const_rgx and can't start
    with '_' in order to be copied.
    '''
    # slightly hacky
    g = globals()
    for attr in dir(module):
        if not attr.startswith('_') and _const_rgx.match(attr):
            g[attr] = getattr(module, attr)


#
# IMPLEMENTATION SPECIFIC CONSTANTS
# define these in a separate file and use cfg.load(module)
#

# Should point to a subclass of agent.drone
# Defines the agent to initialize in the environment
AGENT = None

# (optional)
# Defines a method to render the agent in the opengl
# context. See roombasim.graphics.display for examples.
RENDER_AGENT = None

#
# MATH CONSTANTS
#

# please don't change these ;p
PI = np.pi
TAU = np.pi * 2

#
# ROOMBA CONFIGURATION
#

# Speed of the roomba moving forwards
ROOMBA_LINEAR_SPEED = 0.33 # m/s

# Turning speed of the roomba
ROOMBA_ANGULAR_SPEED = 1.279 # rad/s

# Time until a full reverse (milliseconds)
ROOMBA_REVERSE_PERIOD = 20000

# Time until stopped Obstacle Roomba checks to see if it can move (milliseconds)
ROOMBA_OBSTACLE_STOP_PERIOD = 50

# Time until random heading noise is applied (milliseconds)
ROOMBA_HEADING_NOISE_PERIOD = 5000
ROOMBA_OBSTACLE_NOISE_PERIOD = 1000

# Maximum heading noise (applied in either direction) in radians
ROOMBA_HEADING_NOISE_MAX = 20 * (np.pi / 180)
ROOMBA_OBSTACLE_NOISE_MAX = 23 * (np.pi / 180)

# Turn radius for obstacle roombas
ROOMBA_OBSTACLE_TURN_RADIUS = 3
ROOMBA_TARGET_TURN_RADIUS = 1

# Python doesn't have enums...
ROOMBA_STATE_IDLE = 0
ROOMBA_STATE_FORWARD = 1
ROOMBA_STATE_TURNING = 2
ROOMBA_STATE_OOB = 3 #out of bounds

# Roomba's radius in meters
ROOMBA_RADIUS = 0.35 / 2

#Sensor Pad Radius for the roomba
SENSOR_PAD_RADIUS = .34/2
PAD_HEIGHT = .04

#Roomba height in meters
ROOMBA_HEIGHT = .1

#Obstacle pole radius in meters
OBSTACLE_POLE_RADIUS = .05



# Drone Rotor Radius
DRONE_RADIUS = .258
ROTOR_OFFSET = .15

XY_VEL = .8 #Typical velocity in Meters per second
Z_VEL = .4
LAND_IN_FRONT_DIST = .5
ROOMBA_HEIGHT = .1


#Angle of bottom camera field of view, in radians
BOTTOM_CAMERA_FOV = 2*PI/3

BOUND = 20.0

def getObstacleHeight():
    return np.random.uniform(.05, 2)

