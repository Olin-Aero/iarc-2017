import f_config as cfg
from f_filter import *
from scipy.stats import mvn

# TODO : incorporate covariances into matching
# expose covariance initialization

class Particle(object):
    ID_INVALID = -1
    SIGMAS = cfg.SIGMAS
    @staticmethod
    def set_sigmas(sigmas):
        Particle.SIGMAS = sigmas
    def __init__(
            self, pose,
            t0=0.0,
            t=cfg.T_NULL):
        """
        pose = np.array(shape=5, dtype=np.float32)
        t0 = observation time
        t = type, enum(int)
        """
        if len(pose) == 3:
            self._pose = np.zeros(5, dtype=np.float32)
            self._pose[:3] = pose[:3]
        else:
            self._pose = np.copy(pose)

        self._t0 = t0 # initial observation time

        self._t = t # particle type

    def __repr__(self):
        r = {
                'pose' : self._pose,
                't0' : self._t0,
                't' : self._t,
                }
        return str(r)
    def __str__(self):
        return repr(self)
    def clone(self):
        return Particle(
                self._pose.copy(),
                self._t0,
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
        p1 = self.as_vec()
        p2 = p2.as_vec()
        d = np.abs(ukf_residual(p1,p2))
        cov = np.diag(self.SIGMAS[:3])
        # assume independent x-y-t
        p, _ = mvn.mvnun(d, [20,20,np.pi], np.zeros_like(d), cov)

        return p*(2 ** 3) # account for quartiles
    def as_vec(self):
        return np.copy(self._pose[:3])

def check(t0, t1, int0, int1):
    return (t0%int0 > int1) and (t1%int0 < int1)

class Target(Particle):
    def __init__(
            self, pose,
            t0=0.0,
            t=cfg.T_RED):
        """
        Refer to Particle() for parameter definition.
        """
        super(Target, self).__init__(
            pose, t0, t)
        self._state = cfg.S_RUN

    def step(self, t, dt):

        # evolve pose ...
        (x,y,h,v,w) = self._pose
        x += (v * np.cos(h) * dt)
        y += (v * np.sin(h) * dt)
        h += (w * dt)
        self._pose[:] = [x,y,h,v,w]

        # state transition
        t1 = t+dt
        next_state = self._state
        if t1 % cfg.INT_REVERSE < cfg.T_180:
            next_state = cfg.S_TURN
        elif t1 % cfg.INT_NOISE < cfg.T_NOISE:
            next_state = cfg.S_NOISE
        else:
            next_state = cfg.S_RUN

        if self._state != next_state:
            # initialize state
            if next_state == cfg.S_TURN:
                self._pose[3] = 0.0
                self._pose[4] = np.random.normal(np.pi/cfg.T_180, self.SIGMAS[4]*dt)
            elif next_state == cfg.S_NOISE:
                self._pose[3] = 0.0
                self._pose[4] = np.random.uniform(-cfg.MAX_NOISE_W, cfg.MAX_NOISE_W)
            elif next_state == cfg.S_RUN:
                self._pose[3] = np.random.normal(0.33, self.SIGMAS[3]*dt)
                self._pose[4] = np.random.normal(0.0, self.SIGMAS[4]*dt)
            self._state = next_state


class SimpleParticle(Particle):
    def __init__(
            self, pose,
            t0=0.0,
            t=cfg.T_SIMP):
        """
        Refer to Particle() for parameter definition.
        """
        super(SimpleParticle, self).__init__(
                pose, t0, t)
    def step(self, t, dt):
        self._pose.step(dt)

class Drone(Particle):
    def __init__(
            self, pose,
            t0=0.0,
            t=cfg.T_DRONE):
        """
        Refer to Particle() for parameter definition.
        """
        super(Drone, self).__init__(
                pose, t0, t)

class UKFEstimate(Particle):
    def __init__(
            self, pose,
            t0=0.0,
            t=cfg.T_NULL,
            ukf=None
            ):
        super(UKFEstimate, self).__init__(
                pose, t0, t)
        self.ukf = ukf
    def predict(self, t, dt, obs=False):
        # predict ...
        # obs = observability
        self.ukf.P = (self.ukf.P + self.ukf.P.T) / 2.0
        self.ukf.Q = dt * self.ukf._Q

        self.ukf.predict(dt, fx_args=(t,obs))
        self._pose = self.ukf.x.copy()
    def update(self, pose):
        self.ukf.update(pose)
    def p(self):
        # TODO : hard-coded error margin
        cov = self.ukf.P[:3,:3] # ignore velocity components ( for now )
        # error margin, +- 0.5m, 30 deg.
        d = np.asarray([0.5, 0.5, np.deg2rad(30)])
        _p, _ = mvn.mvnun(
            -d, d,
            np.zeros_like(d),
            cov
            )
        return _p
    def cov(self):
        return self.ukf.P[:2,:2]

class ObservationParticle(Particle):
    def __init__(
            self, pose,
            t0=0.0,
            t=cfg.T_NULL
            ):
        super(ObservationParticle, self).__init__(
                pose, t0, t)
    def as_vec(self):
        return np.copy(self._pose[:3])
