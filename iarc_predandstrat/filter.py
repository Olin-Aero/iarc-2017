import numpy as np
from scipy.stats import mvn

"""
TODO : handle uncertainty as prob. or cov?
"""

## Enums
T_TARG, T_OBST = range(2)
C_RED, C_GREEN = range(2)

SIGMA_X = 0.1 # 10 cm
SIGMA_Y = 0.1 # 10 cm
SIGMA_T = 0.34 #20 deg.
KEEP_THRESH = 0.25 # threshold for keeping particles
P_DECAY = 0.87 # 1.0 -> 0.25 after 10 sec.

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
    def __init__(
            self, id, pose,
            p0=1.0, t0=0.0,
            t=T_TARG, c=None):

        self._id = id
        self._pose = pose

        self._p0 = p0 # initial observation probability
        self._t0 = t0 # initial observation time

        self._t = t # particle type
        self._c = c # color

    def sense(self, drone, noise=True):
        # this is only to be used with GT particles
        if drone.visible(self._pose):
            if noise:
                return add_noise(self._pose)
            else:
                return self._pose
        else:
            return None

    def match(self, p2):
        # TODO : vectorize?
        # TODO : incorporate velocities
        # TODO : consider color[r/g] and type[t/o], when provided
        p1  = self.as_vec()
        p2  = p2.as_vec()
        d   = np.abs(np.subtract(p1, p2))
        cov = np.diag([SIGMA_X**2, SIGMA_Y**2, SIGMA_T**2])
        # assume independent x-y-t
        p, _ = mvn.mvnun(
                -d, d,
                np.zeros_like(d),
                cov
                )
        return (1-p) # outer-probability

    def __eq__(self, p):
        return self._id != Particle.ID_INVALID and \
                p._id != Particle.ID_INVALID and \
                self._id == p._id
    def as_vec(self):
        # TODO : add velocity components
        p = self._pose
        return np.asarray([p._x, p._y, p._t])
    def p(self, t):
        return self._p0 * P_DECAY ** (t-_t0)

class Target(Particle):
    def __init__(self, pose, id, p=1.0):
        super(Target, self).__init__(pose, id, p)
    def step(self, t, dt):
        if (t+dt)%5.0 < t%5.0:
            pass

def particle_filter(rs, obs, obs_rs, t):
    # rs = {id : [roombas]}
    # obs_rs = [roombas]

    # heuristic : 
    # clear particles when positional variance is too big
    # clear particles when observable and not observed
    # clear particles when "stale" -- long time since observation

    _rs = {k:[] for k in rs.keys()}

    for (ps_i, ps_rs) in rs.iteritems():
        for ps_r in ps_rs:
            if ps_r.p(t) < KEEP_THRESH: # stale
                continue
            if ps_r in obs:
                # visible
                vals = [ps_r.match(obs_r) for obs_r in obs_rs]
                p = np.max(vals)
                if p > KEEP_THRESH:
                    # refresh
                    ps_r._t0 = t
                    ps_r._p0 = p
                    _rs[ps_i].append(ps_r)
            else:
                # invisible, passthrough
                _rs[ps_i].append(ps_r)

    #assert(len(obs) == len(obs_rs))
    n,m = len(rs), len(obs)

    # match-values
    val = np.zeros((n,m), dtype=np.float32)
    for i in range(n):
        for j in range(m):
            val[_i,_j] = float(obs_rs[_j] in obs) * \
                    match_particle(rs[_i], obs_rs[_j])

    # observation assignment
    asn_idx = np.argmax(val, axis=-1)
    asn_val = val[range(len(val)), asn_idx]

    # apply simple threshold
    unseen_mask = (asn_val <= 0)
    seen_mask = (asn_val >= P_THRESH)
    good_mask = np.logical_or(unseen_mask, seen_mask)

    # resolve ambiguity
    u, c = np.unique(asn_idx, return_counts=True)
    for d in u[c>1]:
        ids = [r.id for r in rs[asn_idx == d]]
        same_id = np.all(np.equal(ids, ids[0]))
        if not same_id:
            one_good = np.sum(asn_val
            if (
            if asn_val
            asn_val

    # in-between : "bad" particles

    rs = rs[good_mask]

    unseen = rs[asn_val <= 0]
    unmatched = rs[np.logi

    # mask duplicates
    for d in u[c>1]:
        ids = []
        for idx in np.where(asn_idx == d):
            ds.append(rs[idx].id)
        rs[rs==d]
    d = u[c>1]
    d_mask = asn_idx[asn_idx == u[c > 1]]

    rs_ = [] # new rs

    for r in rs:
        if not (r in obs):
            # persist
            # if not stale(r):
            # TODO : decrease p?
            rs_.append(r)
            continue
        # clear-or-modify
        p_max = 0.0
        r_best = None
        for o_r in obs_rs:
            p = match_particle(r, o_r)
            if p > p_max:
                r_best = o_r
        if p_max > P_THRESH:
            r_best.id = r.id
            rs_.append(r_best)
        else:
            rs_.append(r)

def render(
        drone,
        roombas,
        particles):
    pass

class Drone(object):
    def __init__(self, pose):
        pass


def main():
    drone = Drone(Pose())
    targets = [Target(Pose.random(), ('target%d'%i)) for _ in range(8)]


if __name__ == "__main__":
    main()
