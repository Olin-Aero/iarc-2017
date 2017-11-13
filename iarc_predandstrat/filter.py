import numpy as np
from scipy.stats import mvn
from scipy.optimize import linear_sum_assignment
from scipy.linalg import sqrtm
import cv2
import itertools
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

"""
TODO : handle uncertainty as prob. or cov?
"""

## Enums
T_TARG, T_OBST, T_SIMP = range(3)
C_RED, C_GREEN = range(2)

SIGMA_X = 0.1 # 10 cm
SIGMA_Y = 0.1 # 10 cm
SIGMA_T = 0.34 #20 deg.
SIGMA_V = 0.01 # 0.1 m/s
SIGMA_W = 0.03 # 0.3 rad/s

KEEP_THRESH = 0.25 # threshold for keeping particles
MATCH_THRESH = 0.5
P_DECAY = 0.87 # 1.0 -> 0.25 (KEEP_THRESH) after 10 sec.

## Utility
def add_noise(data, s):
    z = np.random.normal(scale=s)
    #return data + z
    msk = (np.abs(z) >= 2*s)
    while np.any(msk):
        z[msk] = np.random.normal(scale=s[msk])
        msk = (np.abs(z) >= 2*s)
    return data + z

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
        #d   = np.abs(np.subtract(p1, p2))
        d = np.abs(ukf_residual(p1,p2))
        cov = np.diag([SIGMA_X**2, SIGMA_Y**2, SIGMA_T**2])
        # assume independent x-y-t
        p, _ = mvn.mvnun(
                -d, d,
                np.zeros_like(d),
                cov
                )
        return (1 - p) # outer-probability

    def cost(self, p2):
        p1 = self.as_vec()
        p2 = p2.as_vec()
        delta = np.abs(ukf_residual(p1,p2))
        weights = [1.0, 1.0, 0.01]
        return np.linalg.norm(np.multiply(delta, weights))

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
        self._pose.step(dt)

def ukf_filter(ukfs, est, obs, t, dt, ar=None):
    n = len(est)
    m = len(obs)
    #print 'n,m', n,m

    prob = np.zeros(shape=(n,m), dtype=np.float32)
    cost = np.zeros(shape=(n,m), dtype=np.float32)

    # predict from dt
    if t > 0:
        for i in est.keys():
            ukfs[i].predict(dt)
            est[i]._pose._data = ukfs[i].x.copy()
            # TODO : covariance -> probability?

    # assign
    for i in est.keys():
        for j, o in enumerate(obs):
            prob[i,j] = est[i].match(o)
            cost[i,j] = est[i].cost(o)

    cost = (1.0 - prob)

    #for i in est.keys():
    #    for j, o in enumerate(obs):
    #        p_i = est[i]._pose._data[:2]
    #        p_j = o._pose._data[:2]
    #        d = (p_i - p_j)
    #        cost[i,j] = np.sqrt(np.sum(d**2))

    #cost = 1.0 - prob
    #cost = (1.0 - prob)**2
    #cost = (cost * 100).astype(np.int32) / 100.

    #oob = [(est[i]._pose not in ar) for i in est.keys()]
    #cost[oob] += 9999
    #print cost

    # i.e. est[i] ~ obs[j]

    # update
    i_idx, j_idx = linear_sum_assignment(cost)
    #print i_idx, j_idx

    clear_i = np.ones(n)
    add_j = np.ones(m)

    for (i,j) in zip(i_idx, j_idx):
        if ar and (est[i]._pose in ar):
            if (prob[i,j] > MATCH_THRESH):
                try:
                    ukfs[i].update(obs[j]._pose._data[:3])
                    est[j]._p0 = prob[i,j]
                    est[j]._t0 = t
                    clear_i[i] = False
                    add_j[j] = False
                except Exception as e:
                    print e
                    print obs[j]._pose._data[:3]
            else:
                pass
        else:
            pass

        est[i]._pose._data = ukfs[i].x

    # back to particles

def particle_filter(rs, obs, obs_rs, t):
    # rs = {id : [roombas]}
    # obs_rs = [roombas]

    # heuristic : 
    # clear particles when positional variance is too big
    # clear particles when observable and not observed
    # clear particles when "stale" -- long time since observation

    _rs = {k:[] for k in rs.keys()}

    old = []
    for (ps_i, ps_r) in rs.iteritems():
        #for ps_r in ps_rs:
            if ps_r.p(t) < KEEP_THRESH: # stale
                continue
            if ps_r._pose in obs:
                # visible
                vals = [ps_r.match(obs_r) for obs_r in obs_rs]

                #print ' ====== '
                #print ps_r._pose
                #print [obs_r._pose for obs_r in obs_rs]
                #print vals

                p = np.max(vals)
                #print ps_i, np.argmax(vals), p
                old.append(np.argmax(vals))
                if p > KEEP_THRESH:
                    # refresh
                    ps_r._t0 = t
                    ps_r._p0 = p
                    _rs[ps_i].append(ps_r)
            else:
                # invisible, passthrough
                _rs[ps_i].append(ps_r)
    print np.unique(old)

    # TODO : generate new particles from observation
    # TODO : generate new particles from state prediction?
    # i.e. collision / t-noise, etc.

    return _rs

class Renderer(object):
    def __init__(self, w=500, h=500, s=25):
        self._w = w
        self._h = h
        self._s = s
        self._img = np.zeros(shape=(w,h,4), dtype=np.uint8)
    def _convert(self, x, y):
        x = self._w/2 + self._s*x
        y = self._h/2 + self._s*(-y)
        return (int(x),int(y))
    def __call__(self,
            drone,
            roombas,
            particles,
            t,
            delay=10
            ):

        img = self._img.copy()
        img_p = self._img.copy()
        img_r = self._img.copy()

        # roombas
        for r in roombas:
            x, y = self._convert(r.x, r.y)
            vx, vy = 20*np.cos(r.t), -20*np.sin(r.t)
            vx, vy = (int(e) for e in (vx,vy))
            cv2.circle(img,
                    (x,y),
                    10,
                    color=(255,255,0),
                    thickness=-1
                    )
            cv2.line(img,
                    (x,y),
                    (x+vx,y+vy),
                    (255,255,0),
                    4
                    )

        # particles
        for p in particles:
            #print 'p', p.p(t)
            x, y = self._convert(p._pose.x, p._pose.y)
            vx, vy = 10*np.cos(p._pose.t), -10*np.sin(p._pose.t)
            vx, vy = (int(e) for e in (vx,vy))
            cv2.circle(img,
                    (x,y),
                    5,
                    (255,0,0, int(255 * p.p(t))),
                    -1
                    )
            cv2.line(img,
                    (x,y),
                    (x+vx,y+vy),
                    (255,0,0),
                    2
                    )

        # drone
        # cv2.circle(img,
        #         self._convert(drone.x, drone.y),
        #         20,
        #         (0,0,255),
        #         -1
        #         )

        cv2.imshow('world', img)
        if cv2.waitKey(delay) == 27:
            return True
        return False

class Drone(object):
    def __init__(self, pose):
        self._pose = pose
        pass

def ukf_hx(x):
    return x[:3]

def ukf_fx(x0, dt):
    # TODO : support time-based transition?
    # or handle it outside of ukf by manipulating state
    # I think fx_args in predict() can work?

    # x-y-t-v-w
    x,y,t,v,w = x0
    x += v * np.cos(t) * dt
    y += v * np.sin(t) * dt
    t += w * dt
    x = np.asarray([x,y,t,v,w])
    return x

def ukf_mean(xs, wm):
    # Important : SUM! not mean.
    mx = np.sum(xs * np.expand_dims(wm, -1), axis=0)
    ms = np.mean(np.sin(xs[:,2])*wm)
    mc = np.mean(np.cos(xs[:,2])*wm)
    mx[2] = np.arctan2(ms,mc)
    return mx

def ukf_residual(a,b):
    d = np.real(np.subtract(a,b))
    # sometimes gets imag for some reason
    d[2] = np.arctan2(np.sin(d[2]), np.cos(d[2]))
    return d

def main():
    # parameters
    n_targets = 14
    n_particles = 1 # particles per target
    dt = 0.1
    steps = (100.0 / dt)
    sigmas = np.asarray([SIGMA_X, SIGMA_Y, SIGMA_T, SIGMA_V, SIGMA_W])

    # initialization
    render = Renderer()
    drone = Drone(Pose())
    targets = [SimpleParticle(Pose.random()) for i in range(n_targets)]
    mws = MerweScaledSigmaPoints(5,1e-3,2,-2,sqrt_method=sqrtm,subtract=ukf_residual)

    ukf_args = {
            'dim_x' : 5,
            'dim_z' : 3, #x-y-t
            'dt' : dt,
            'hx' : ukf_hx,
            'fx' : ukf_fx,
            'points' : mws,
            'x_mean_fn' : ukf_mean,
            'z_mean_fn' : ukf_mean,
            'residual_x' : ukf_residual,
            'residual_z' : ukf_residual
            }

    # TODO : arbitrary covariances
    # TODO : dt necessary?
    P = np.diag(np.square(sigmas)) # covariance
    R = dt * np.diag(np.square([SIGMA_X,SIGMA_Y,SIGMA_T])) # measurement noise
    Q = dt * np.diag(np.square([0.02, 0.02, np.deg2rad(3), 0.03, 0.03])) # process noise
    #Q = np.diag(([0.02, 0.02, np.deg2rad(3), 0.03, 0.03])) # process noise

    # TODO : testing with SimpleParticle, not Target

    # initialize particles
    particles = {}
    ukfs = {}

    for i, t in enumerate(targets):
        for k in range(n_particles):
            # initialize with noisy pose, 3 per particle
            # particles[i].append(SimpleParticle(pose))
            pose = Pose(*add_noise(t._pose._data, sigmas))
            #pose = t._pose.clone()
            particles[i] = SimpleParticle(pose)
            ukfs[i] = UKF(**ukf_args)
            ukfs[i].Q = Q.copy() 
            ukfs[i].R = R.copy() 
            ukfs[i].x = pose._data.copy()
            ukfs[i].P = P.copy() 

    obs = CircularObservation(
            drone._pose.x,
            drone._pose.y,
            20.0 # TODO : arbitrary, fix
            )

    for t in np.linspace(0.0, 10.0, steps):
        obs_rs = []
        for r in targets:
            if r._pose in obs:
                obs_r = r.clone()
                obs_r._pose = Pose(*add_noise(r._pose._data, s=dt*sigmas))
                #obs_r._pose = r._pose.clone()
                obs_rs.append(obs_r)

        stop = render(
                drone._pose,
                [_t._pose for _t in targets],
                particles.values(),
                t,
                delay=1
                )

        ukf_filter(ukfs, particles, obs_rs, t, dt, ar=obs)

        for _t in targets:
            _t.step(t,dt)

        if stop:
            break

if __name__ == "__main__":
    main()
