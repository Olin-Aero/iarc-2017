import numpy as np
from scipy.stats import mvn

"""
TODO : handle uncertainty as prob. or cov?
"""

## Enums
T_TARG, T_OBST, T_SIMP = range(3)
C_RED, C_GREEN = range(2)

SIGMA_X = 0.1 # 10 cm
SIGMA_Y = 0.1 # 10 cm
SIGMA_T = 0.34 #20 deg.
KEEP_THRESH = 0.25 # threshold for keeping particles
P_DECAY = 0.87 # 1.0 -> 0.25 (KEEP_THRESH) after 10 sec.

## Utility
def add_noise(data, s=1.0):
    return np.random.normal(loc=data,scale=s)

## Observations
class Observation(object):
    def __init__(self):
        pass
    def __contains__(self, item):
        raise NotImplementedError("Base Class!")

class CircularObservation(object):
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r
    def __contains__(self, item):
        dx = item.x - self.x
        dy = item.y - self.y
        return (dx**2+dy**2 < self.r**2)

## Pose
class Pose(object):
    def __init__(self,
            x=0,y=0,t=0,
            v=0,w=0):
        self._data = np.asarray([x,y,t,v,w], dtype=np.float32)
    def __repr__(self):
        r = {
                'x':self.x,
                'y':self.y,
                't':self.t,
                'v':self.v,
                'w':self.w
                }
        return str(r)
    def __str__(self):
        return repr(self)
    @property
    def x(self):
        return self._data[0]
    @x.setter
    def x(self,_x):
        self._data[0]=_x
    @property
    def y(self):
        return self._data[1]
    @y.setter
    def y(self,_y):
        self._data[1]=_y
    @property
    def t(self):
        return self._data[2]
    @t.setter
    def t(self,_t):
        self._data[2]=_t
    @property
    def v(self):
        return self._data[3]
    @v.setter
    def v(self,_v):
        self._data[3]=_v
    @property
    def w(self):
        return self._data[4]
    @w.setter
    def w(self,_w):
        self._data[4]=_w
    @staticmethod
    def random():
        x = np.random.uniform(-10.0, 10.0)
        y = np.random.uniform(-10.0, 10.0)
        t = np.random.uniform(-np.pi, np.pi)
        v = np.random.normal(0.27, 0.33)
        w = np.random.normal(0.8, 1.2)
        return Pose(x,y,t,v,w)
    def step(self, dt):
        self.x += (self.v * np.cos(self.t) * dt)
        self.y += (self.v * np.sin(self.t) * dt)
        self.t += (self.w * dt)
    def clone(self):
        return Pose(*self._data)
            
class Particle(object):
    ID_INVALID = -1
    def __init__(
            self, pose,
            p0=1.0, t0=0.0,
            t=T_TARG, c=None):
        self._pose = pose

        self._p0 = p0 # initial observation probability
        self._t0 = t0 # initial observation time

        self._t = t # particle type
        self._c = c # color

    def __repr__(self):
        r = {
                'pose' : self._pose,
                'p0' : self._p0,
                't0' : self._t0,
                't' : self._t,
                'c' : self._c
                }
        return str(r)
    def __str__(self):
        return repr(self)
    def clone(self):
        return Particle(
                self._pose.clone(),
                self._p0, self._t0,
                self._t, self._c)
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
    def as_vec(self):
        # TODO : add velocity components
        p = self._pose
        return np.asarray([p.x, p.y, p.t])
    def p(self, t):
        return self._p0 * P_DECAY ** (t-self._t0)

class Target(Particle):
    def __init__(
            self, pose,
            p0=1.0, t0=0.0,
            t=T_TARG, c=None):
        super(Target, self).__init__(
            pose, p0, t0, t, c)
    def step(self, t, dt):
        if (t+dt)%5.0 < t%5.0:
            pass

class SimpleParticle(Particle):
    def __init__(
            self, pose,
            p0=1.0, t0=0.0,
            t=T_SIMP, c=None):
        super(SimpleParticle, self).__init__(
                pose, p0, t0, t, c)
    def step(self, t, dt):
        self.pose.step(dt)

def particle_filter(rs, obs, obs_rs, t):
    # rs = {id : [roombas]}
    # obs_rs = [roombas]

    # heuristic : 
    # clear particles when positional variance is too big
    # clear particles when observable and not observed
    # clear particles when "stale" -- long time since observation

    _rs = {k:[] for k in rs.keys()}

    old = []
    for (ps_i, ps_rs) in rs.iteritems():
        for ps_r in ps_rs:
            if ps_r.p(t) < KEEP_THRESH: # stale
                continue
            if ps_r._pose in obs:
                # visible
                vals = [ps_r.match(obs_r) for obs_r in obs_rs]

                print ' ====== '
                print ps_r._pose
                print [obs_r._pose for obs_r in obs_rs]
                print vals

                p = np.max(vals)
                print ps_i, np.argmax(vals), p
                old.append(np.argmax(vals))
                if p > KEEP_THRESH:
                    # refresh
                    ps_r._t0 = t
                    ps_r._p0 = p
                    _rs[ps_i].append(ps_r)
            else:
                # invisible, passthrough
                _rs[ps_i].append(ps_r)

    # TODO : generate new particles from observation
    # TODO : generate new particles from state prediction?
    # i.e. collision / t-noise, etc.

    return _rs

def render(
        drone,
        roombas,
        particles):
    pass

class Drone(object):
    def __init__(self, pose):
        self._pose = pose
        pass

def main():
    drone = Drone(Pose())
    targets = [SimpleParticle(Pose.random()) for i in range(8)]
    # TODO : testing with SimpleParticle, not Target

    # initialize particles
    particles = {i:[] for i in range(8)}
    sigmas = [SIGMA_X, SIGMA_Y, SIGMA_T, 0.0, 0.0]
    for i, t in enumerate(targets):
        for k in range(3):
            # initialize with noisy pose, 3 per particle
            pose = Pose(*add_noise(t._pose._data, sigmas))
            particles[i].append(SimpleParticle(pose))

    obs = CircularObservation(
            drone._pose.x,
            drone._pose.y,
            5.0 # TODO : arbitrary, fix
            )

    dt = 0.01
    #steps = (10.0 / dt)
    steps = 100
    for t in np.linspace(0.0, 10.0, steps):

        obs_rs = []
        for r in targets:
            if r._pose in obs:
                obs_r = r.clone()
                obs_r._pose = Pose(*add_noise(r._pose._data))
                obs_rs.append(obs_r)

        particles = particle_filter(particles, obs, obs_rs, t)

if __name__ == "__main__":
    main()
