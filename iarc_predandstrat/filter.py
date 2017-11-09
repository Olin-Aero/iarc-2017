import numpy as np

"""
TODO : handle uncertainty as prob. or cov?
"""

## Enums
T_TARG, T_OBST = range(2)
C_RED, C_GREEN = range(2)

## Utility
def add_noise(data, s=1.0):
    return np.random_normal(loc=data,scale=s)

## Observations
class Observation(object):
    def __init__(self):
        pass
    def __contains__(self, item):
        raise NotImplementedError("Base Class!")

class CircularObservation(object):
    def __init__(self, x, y, r):
        self._x = x
        self._y = y
        self._r = r
    def __contains__(self, item):
        dx = item._x - self._x
        dy = item._y - self._y
        return (dx**2+dy**2 < self._r**2)

## Pose
class Pose(object):
    def __init__(self,
            x=0,y=0,t=0,
            v=0,w=0):
        self._x = x
        self._y = y
        self._t = t
        self._v = v
        self._w = w
    @staticmethod
    def random():
        x = np.random.uniform(-10.0, 10.0)
        y = np.random.uniform(-10.0, 10.0)
        t = np.random.uniform(-np.pi, np.pi)
        v = np.random.normal(0.27, 0.33)
        w = np.random.normal(0.8, 1.2)
        return Pose(x,y,t,v,w)
            
class Particle(object):
    ID_INVALID = -1
    def __init__(self, pose, id, p=1.0, t=T_TARG, c=None):
        self._id = id
        self._pose = pose
        self._p = p # probability? covariance?
        self._t = t # particle type
        self._c = c # color
        # may be changed into covariances
    def sense(self, drone, noise=True):
        if drone.visible(self._pose):
            if noise:
                return add_noise(self._pose)
            else:
                return self._pose
        else:
            return None
    def match(self, p):
        m_t = self._t == p._t
        #criteria : x-y-t-v-w
        pass
    def __eq__(self, p):
        return self._id != Particle.ID_INVALID and \
                p._id != Particle.ID_INVALID and \
                self._id == p._id

class Target(Particle):
    def __init__(self, pose, id, p=1.0):
        super(Target, self).__init__(pose, id, p)
    def step(self, t, dt):
        if (t+dt)%5.0 < t%5.0:
            pass


def particle_filter(rs, obs, obs_rs):
    rs_ = [] # new rs
    for r in rs:
        if r in obs: # is included in observation
            np.argmax(r, obs_rs)
            # clear-or-modify
            pass
        else:
            # persist
            # if not stale(r):
            # TODO : decrease p?
            rs_.append(r)

def render(
        drone,
        roombas,
        particles):
    pass

class Drone(object):
    def __init__(self, pose):
        pass
