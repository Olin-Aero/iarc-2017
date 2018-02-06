
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
        self.P[3,3] = 100.0
        self.P[4,4] = 100.0
        self.R = np.diag(np.square([0.1, 0.1, np.deg2rad(5)])) # measurement noise: 5cm/5deg err.

        # process noise
        #G = [0.5*dt**2, 0.5*dt**2, 0.5*dt**2, dt, dt] # acceleration-noise model
        self.Q = dt * np.diag([0.02, 0.02, np.deg2rad(3), 0.01, 0.01])
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

        self.p_idx = 0
        self.est = {}

    def create(self, pose, *args, **kwargs):
        ukf = UKF(**self.ukf_args)
        ukf.Q = self.Q.copy()
        ukf.R = self.R.copy()
        ukf.x = np.copy(pose)
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
            # TODO : Observation flag assignment somewhere
            e.predict(t, dt, obs=True)#(t > e._t0 + 1.0))
            # TODO(yoonyoungcho) : arbitrary time threshold
            # only start simulational iteration
            # if it has been more than a second since observation.

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
            print dt
            e.predict(t, dt, obs=(e._pose in obs_ar))

        # assign observations
        for k in est.keys():
            for j, o in enumerate(obs):
                prob[k2i[k],j] = est[k].match(o)
                cost[k2i[k],j] = est[k].cost(o)
        i_idx, j_idx = linear_sum_assignment(cost)

        # update
        # collect "new" particles
        k_clear = []
        add_obs = np.ones(m)

        for (i,j) in zip(i_idx, j_idx):
            k = i2k[i]
            if est[k]._pose in obs_ar:
                if (prob[i,j] > P_MATCH):
                    #try:
                    est[k].update(obs[j]._pose[:3])
                    # TODO : p0-t0 necessary?
                    est[k]._p0 = prob[i,j]
                    est[k]._t0 = t
                    add_obs[j] = False
                    #except Exception as e:
                    #    print e
                    #    print obs[j]._pose[:3]
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
                j = np.argmax(prob[i])
                if (j not in j_idx) and (np.max(prob[i]) > P_CLEAR):
                    # this was best match
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

if __name__ == "__main__":
    dt = 0.01
    sigmas = np.asarray([S_X, S_Y, S_T, S_V, S_W])
    manager = UKFManager(dt, sigmas) 
