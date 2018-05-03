"""
Filter Configurations,
as well as simulation configurations.
"""
import numpy as np

# Roomba Type/Color Definitions
T_NULL, T_DRONE, T_RED, T_GREEN, T_OBST, T_SIMP = range(6)

# Standard Deviations for pose estimation
S_X, S_Y, S_T, S_V, S_W = (0.1, 0.1, 0.34, 0.1, 0.3)
SIGMAS = np.asarray([S_X, S_Y, S_T, S_V, S_W])

# Probability Values
P_KEEP = 0.25 # particle-keep threshold
P_MATCH = 0.25 # particle-match threshold for update
P_CLEAR = 0.33 # particle-clear threshold
P_DECAY = 0.87 # 1.0 -> 0.25 after 10 sec.

# Known Roomba parameters
MAX_NOISE_W = 0.41

# Event Intervals
INT_NOISE=5.0
INT_REVERSE=20.0

# Action Intervals
T_180=2.15
T_45=T_180/4
T_NOISE=0.85

# Roomba States
S_WAIT, S_RUN, S_NOISE, S_TURN, S_END = range(5)

# Additional Constants ...

T_OBS = 0.5
# TODO: arbitrary; wait 500ms until observation flag is cleared
