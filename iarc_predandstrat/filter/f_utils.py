import numpy as np

from f_pose import *
from f_obs import *
from f_render import *
from f_filter import *
from f_particle import *

def add_noise(x, s):
    """
    Adds noise sampled from truncated-normal distribution.
    i.e. the noise will be within 2 standard deviations.

    Parameters:
    x : original data
    s : standard deviation, scalar or vector-like, same length as data
    """
    z = np.random.normal(scale=s)

    # make sure 2 standard deviations
    msk = (np.abs(z) >= 2*s)
    while np.any(msk):
        z[msk] = np.random.normal(scale=s[msk])
        msk = (np.abs(z) >= 2*s)
    return x + z

