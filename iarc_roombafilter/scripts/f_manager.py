#!/usr/bin/env python2

import numpy as np
from f_utils import *
from f_config import *

from scipy.linalg import sqrtm
from scipy.optimize import linear_sum_assignment
from scipy.stats import mvn

try:
    from filterpy.kalman import UnscentedKalmanFilter as UKF
    from filterpy.kalman import MerweScaledSigmaPoints, JulierSigmaPoints
except ImportError:
    raise ImportError('Unable to import filterpy: Did you forget to run \n\
    "pip install -r $(rospack find iarc_main)/../requirements.txt"')

class UKFManager(object):
    def __init__(self, dt, sigmas):
        # note : apparently Merwe may produce non PSD matrices.
        # See https://github.com/rlabbe/filterpy/issues/44 
        spts = MerweScaledSigmaPoints(5,1e-3,2,-2,subtract=ukf_residual)
        #spts = JulierSigmaPoints(5, 5-2, sqrt_method=np.linalg.cholesky, subtract=ukf_residual)

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
        #self.P[3,3] = self.P[4,4] = 100

        # TODO : configure R such that it accepts covariance input.
        # TODO : when this happens, in case of missing covariance, handle unreasonable input covariance.
        self.R = np.diag(np.square([0.1, 0.1, np.deg2rad(5)])) # measurement noise: 5cm/5deg err.

        # different developments of process noise model
        # left here for archival / referral purposes when things go wrong,
        # or need modifications.

        #G = [0.5*dt**2, 0.5*dt**2, 0.5*dt**2, dt, dt] # acceleration-noise model
        #self.Q = np.square(np.diag(G) * 8.8)
        # G = np.reshape(G, [-1,1])
        # print G.T
        # self.Q = np.dot(G, np.transpose(G)) * (8.8**2)
        #np.diag([0.05, 0.05, np.deg2rad(5), 0.01, 0.01])
        #self.Q = np.outer(Gt,Gt) * (8.8**2)
        #print self.Q
        #G = [0.05, 0.05, np.deg2rad(5), 0.01, 0.01]
        #~5cm / 5 deg. per second
        #self.Q = np.diag(np.square(G))

        # process noise
        self.Q = np.diag([0.02, 0.02, np.deg2rad(3), 0.01, 0.01])

        self.p_idx = 0
        self.est = {}

    def create(self, pose, *args, **kwargs):
        print 'CREATE!'
        ukf = UKF(**self.ukf_args)
        ukf._Q = self.Q.copy()
        ukf.Q = self.Q.copy()
        ukf.R = self.R.copy()

        ukf.x = pose#np.zeros(5, dtype=np.float32)
        #ukf.x[:3] = pose[:3]

        ukf.P = self.P.copy()
        # TODO : fill in more info, such as color(red/green/unknown), type(target/obs/unknown)
        self.est[self.p_idx] = UKFEstimate(pose,
                *args,
                ukf=ukf,
                **kwargs
                )
        self.p_idx += 1

    def predict(self, t, dt):
        # predict from dt
        for e in est.values():
            # preserves observation flag for T_OBS seconds
            # only start simulational iteration
            # if it has been more than a second since observation.

            # TODO(yoonyoungcho) : apply simulation model
            # if event period (noise/reversal/collision) had been entered,
            # and try to verify that it is under that state.
            # might be worthwhile to have a separate matching function
            # for the "state" of the roomba, as defined by the underlying FSM.

            e.predict(t, dt, obs=(t < e._t0 + cfg.T_OBS))

    def step(self, obs, t, dt, obs_ar):
        est = self.est

        n = len(est)
        m = len(obs)
        # assert(m == len(obs_ar))

        prob = np.zeros(shape=(n,m), dtype=np.float32)

        i2k = est.keys()
        k2i = {i2k[i]:i for i in range(n)}

        # predict from dt
        # TODO : deal with expected behavior ( turn at ... ) vs. predicted behavior (difference in position, etc.)
        for e in est.values():
            e.predict(t, dt, obs=e._pose in obs_ar)

        # assign observations
        for k in est.keys():
            for j, o in enumerate(obs):
                prob[k2i[k],j] = est[k].match(o)

        i_idx, j_idx = linear_sum_assignment(1.0 - prob)

        # update
        # collect "new" particles
        k_clear = []
        add_obs = np.ones(m)

        s = None
        cnt=0

        for (i,j) in zip(i_idx, j_idx):
            cnt += 1
            k = i2k[i]
            if (prob[i,j] > P_MATCH):
                #try:
                s = ('est', est[k].ukf.x, 'obs', obs[j]._pose)
                est[k].update(obs[j]._pose[:3])
                est[k]._t0 = t # reset observation time
                add_obs[j] = False
                print "Successful match 3"
                #except Exception as e:
                #    print e
                #    print obs[j]._pose[:3]
            #print prob[i,j]
        #print 'cnt', cnt
        #print 'Diff', s

        # clear invalid (unobserved) estimates
        if m > 0:
            for k,e in est.iteritems():
                i = k2i[k]
                if (i in i_idx): # index check
                    continue
                if (e._pose not in obs_ar): # area check
                    continue
                j = np.argmax(prob[i])
                if (j not in j_idx) and (np.max(prob[i]) > P_CLEAR):
                    # this was best match
                    continue
                k_clear.append(k)

        # update estimates list
        self.est = {k:v for (k,v) in est.iteritems() if (v.p() > P_KEEP) and (k not in k_clear)}

        # create new particles from observation
        # and make a guess based on current time

        # TODO : sync time with roombas' respective reference time somehow

        new_j = np.where(add_obs)[0]
        for o in [obs[j] for j in new_j]:
            p = np.copy(o._pose)
            v,w = 0,0

            # make initial guess ...
            # WARNING : only applicable for TARGET roombas
            if t % cfg.INT_REVERSE < cfg.T_180:
                v = 0.0
                w = np.pi / T_180
            elif t % cfg.INT_NOISE < cfg.T_NOISE:
                v = 0.0
                w = 0.0
            else:
                v = .33
                w = 0.0
            p[3:] = (v,w)

            self.create(p)
        # TODO : merge particles that are too similar?

    def estimates(self):
        return self.est

if __name__ == "__main__":
    dt = 0.01
    sigmas = np.asarray([S_X, S_Y, S_T, S_V, S_W])
    manager = UKFManager(dt, sigmas) 
