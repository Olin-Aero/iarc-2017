"""
Filter Configurations,
as well as simulation configurations.
"""

import numpy as np

T_DRONE, T_TARG, T_OBST, T_SIMP = range(4)
C_RED, C_GREEN = range(2)
S_X, S_Y, S_T, S_V, S_W = (0.1, 0.1, 0.34, 0.1, 0.3)
P_KEEP = 0.25 # particle-keep threshold
P_MATCH = 0.75 # particle-match threshold for update
P_DECAY = 0.87 # 1.0 -> 0.25 after 10 sec.

