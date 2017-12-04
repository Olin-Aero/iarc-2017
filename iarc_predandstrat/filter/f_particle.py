from f_config import *
from f_filter import *
from scipy.stats import mvn

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
        # TODO : incorporate velocities
        # TODO : consider color[r/g] and type[t/o], when provided
        p1  = self.as_vec()
        p2  = p2.as_vec()
        d = np.abs(ukf_residual(p1,p2))
        cov = np.diag([S_X**2, S_X**2, S_T**2])
        # assume independent x-y-t
        p, _ = mvn.mvnun(
                -d, d,
                np.zeros_like(d),
                cov
                )
        return (1 - p) # outer-probability

    def cost(self, p2):
        # currently cost() is different from f(match())
        # mostly to avoid matching faraway objects with similar headings
        p1 = self.as_vec()
        p2 = p2.as_vec()
        delta = np.abs(ukf_residual(p1,p2))
        weights = [1.0, 1.0, 0.01]
        # TODO : arbitrary weights
        return np.linalg.norm(np.multiply(delta, weights))

    def as_vec(self):
        # TODO : add velocity components
        p = self._pose
        return np.asarray([p.x, p.y, p.t])
    def p(self, t):
        return self._p0 * P_DECAY ** (t-self._t0)

def check(t0, t1, int0, int1):
    return (t0%int0 > int1) and (t1%int0 < int1)

class Target(Particle):
    def __init__(
            self, pose,
            p0=1.0, t0=0.0,
            t=T_TARG, c=None):
        super(Target, self).__init__(
            pose, p0, t0, t, c)
        self._state = S_RUN

    def step(self, t, dt):
        self._pose.step(dt)

        # state transition
        t1 = t+dt
        next_state = self._state
        if t1 % INT_REVERSE < T_180:
            next_state = S_TURN
        elif t1 % INT_NOISE < T_NOISE:
            next_state = S_NOISE
        else:
            next_state = S_RUN

        if self._state != next_state:
            # initialize state
            if next_state == S_TURN:
                self._pose.v = 0.0
                self._pose.w = np.random.normal(np.pi/T_180, S_W*dt)
            elif next_state == S_NOISE:
                self._pose.v = 0.0
                self._pose.w = np.random.uniform(-MAX_NOISE_W, MAX_NOISE_W)
            elif next_state == S_RUN:
                self._pose.v = np.random.normal(0.33, S_V*dt)
                self._pose.w = np.random.normal(0.0, S_W*dt)
            self._state = next_state


class SimpleParticle(Particle):
    def __init__(
            self, pose,
            p0=1.0, t0=0.0,
            t=T_SIMP, c=None):
        super(SimpleParticle, self).__init__(
                pose, p0, t0, t, c)
    def step(self, t, dt):
        self._pose.step(dt)

class Drone(Particle):
    def __init__(
            self, pose,
            p0=1.0, t0=0.0,
            t=T_DRONE, c=None):
        super(Drone, self).__init__(
                pose, p0, t0, t, c)

class UKFEstimate(Particle):
    def __init__(
            self, pose,
            p0=1.0, t0=0.0,
            t=T_TARG, c=None,
            ukf=None
            ):
        super(UKFEstimate, self).__init__(
                pose, p0, t0, t, c)
        self.ukf = ukf
    def predict(self, t, dt, obs):
        # predict ...
        # obs = observability
        self.ukf.predict(dt, fx_args=(t,obs))
        self._pose._data = self.ukf.x.copy()
    def update(self, pose):
        self.ukf.update(pose)
    def p(self):
        # TODO : hard-coded error margin
        cov = self.ukf.P[:3,:3] # ignore velocity components
        # error margin, +- 0.5m
        d = np.asarray([0.5, 0.5, np.deg2rad(30)])
        _p, _ = mvn.mvnun(
            -d, d,
            np.zeros_like(d),
            cov
            )
        return _p
    def cov(self):
        return self.ukf.P[:2,:2]
