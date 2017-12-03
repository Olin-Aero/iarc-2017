from f_utils import *
from f_config import *

import numpy as np
import cv2
from scipy.linalg import sqrtm
from scipy.optimize import linear_sum_assignment
from scipy.stats import mvn
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints, JulierSigmaPoints

class UKFManager(object):
    def __init__(self, dt, sigmas):
        # note : apparently Merwe may produce non PSD matrices.
        # See https://github.com/rlabbe/filterpy/issues/44 
        #spts = MerweScaledSigmaPoints(5,1e-3,2,-2,sqrt_method=sqrtm,subtract=ukf_residual)
        spts = JulierSigmaPoints(5, 5-2, sqrt_method=sqrtm, subtract=ukf_residual)

        self.ukf_args = {
            'dim_x' : 5,
            'dim_z' : 3, #x-y-t
            'dt' : dt,
            'hx' : ukf_hx,
            'fx' : ukf_fx,
            'points' : spts,
            'x_mean_fn' : ukf_mean,
            'z_mean_fn' : ukf_mean,
            'residual_x' : ukf_residual,
            'residual_z' : ukf_residual
            }

        # TODO : arbitrary covariances
        self.P = np.diag(np.square(sigmas))# initial covariance
        self.R = np.diag(np.square([0.05, 0.05, np.deg2rad(5)])) # measurement noise
        # process noise
        Gt = [0.5*dt**2, 0.5*dt**2, 0.5*dt**2, dt, dt] # acceleration-noise model
        G = np.multiply(Gt, [0.01, 0.01, np.deg2rad(5), 0.01, 0.01]) #~1cm / 5 deg. per second
        self.Q = np.diag(np.square(G))

        self.p_idx = 0
        self.est = {}

    def create(self, pose, *args, **kwargs):
        ukf = UKF(**self.ukf_args)
        ukf.Q = self.Q.copy()
        ukf.R = self.R.copy()
        ukf.x = pose._data.copy()
        ukf.P = self.P.copy()
        # TODO : fill in more info, such as color(red/green/unknown), type(target/obs/unknown)
        # TODO : testing with SimpleParticle, not Target
        self.est[self.p_idx] = UKFEstimate(pose,
                *args,
                ukf=ukf,
                **kwargs
                )
        self.p_idx += 1

    def step(self, obs, t, dt, obs_ar):
        est = self.est

        n = len(est)
        m = len(obs)
        # assert(m == len(obs_ar))

        prob = np.zeros(shape=(n,m), dtype=np.float32)
        cost = np.ones(shape=(n,m), dtype=np.float32)

        i2k = est.keys()
        k2i = {i2k[i]:i for i in range(n)}

        # predict from dt
        for e in est.values():
            e.predict(t, dt)

        # assign observations
        for k in est.keys():
            for j, o in enumerate(obs):
                prob[k2i[k],j] = est[k].match(o)
                cost[k2i[k],j] = est[k].cost(o)
        i_idx, j_idx = linear_sum_assignment(cost)
        #print i_idx, j_idx

        # update
        # collect "new" particles
        k_clear = []
        add_obs = np.ones(m)

        for (i,j) in zip(i_idx, j_idx):
            k = i2k[i]
            if est[k]._pose in obs_ar:
                if (prob[i,j] > P_MATCH):
                    #try:
                    est[k].update(obs[j]._pose._data[:3])
                    # TODO : p0-t0 necessary?
                    est[k]._p0 = prob[i,j]
                    est[k]._t0 = t
                    add_obs[j] = False
                    #except Exception as e:
                    #    print e
                    #    print obs[j]._pose._data[:3]
                #print prob[i,j]
            else:
                # no updates ...
                pass

        # clear invalid (unobserved) estimates
        if m > 0:
            for k,e in est.iteritems():
                i = k2i[k]
                if (i in i_idx): # index check
                    continue
                if (e._pose not in obs_ar): # area check
                    continue
                if (np.max(prob[i]) > P_CLEAR):
                    continue
                k_clear.append(k)

        # update estimates list
        self.est = {k:v for (k,v) in est.iteritems() if (v.p() > P_KEEP) and (k not in k_clear)}

        # create new particles from observation
        new_j = np.where(add_obs)[0]
        for o in [obs[j] for j in new_j]:
            self.create(o._pose)
        # TODO : merge particles that are too similar?

    def estimates(self):
        return self.est.values()

def main():
    # parameters
    n_targets = 30
    dt = 0.1
    steps = (100.0 / dt)
    sigmas = np.asarray([S_X, S_Y, S_T, S_V, S_W])

    # initialization
    drone = Drone(Pose())
    targets = [Target(Pose.random()) for i in range(n_targets)]

    # render ...
    win_name = 'world'
    render = Renderer(win_name)
    def set_drone_pos(e,x,y,f,p):
        x,y = render._convert_inv(x,y)
        drone._pose.x=x
        drone._pose.y=y
    cv2.setMouseCallback(win_name,set_drone_pos)

    # initialize filter manager
    manager = UKFManager(dt, sigmas)
    particles = {}

    for i, t in enumerate(targets):
        # initial estimates with noisy pose
        pose = Pose(*add_noise(t._pose._data, sigmas))
        manager.create(pose)

    t = 0
    while True:

        # observations ...
        obs_ar = CircularObservation(
                drone._pose.x,
                drone._pose.y,
                3.0 # TODO : arbitrary, fix
                )
        obs = []
        for r in targets:
            if r._pose in obs_ar:
                o = r.clone()
                o._pose = Pose(*add_noise(r._pose._data, s=dt*sigmas))
                obs.append(o)

        # render
        k = render(
                drone._pose,
                [_t._pose for _t in targets],
                manager.estimates(),
                t,
                delay=100
                )

        # filter
        manager.step(obs, t, dt, obs_ar)

        # step world
        for _t in targets:
            _t.step(t,dt)

        if k == 27:
            break
        t += dt

if __name__ == "__main__":
    main()
