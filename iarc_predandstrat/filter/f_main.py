from f_utils import *
from f_config import *

import numpy as np
import cv2
from scipy.linalg import sqrtm
from scipy.optimize import linear_sum_assignment
from scipy.stats import mvn
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

#def cov2p(x):
#    # WARNING : experimental
#    d = np.diag(x)
#    d = np.abs(ukf_residual(d, np.zeros_like(d)))
#    d = d[:3]
#    cov = np.diag([S_X**2, S_X**2, S_T**2])
#    p, _ = mvn.mvnun(
#            -d, d,
#            np.zeros_like(d),
#            cov
#            )
#    return (1 - p) # outer-probability

def cov2p(cov):
    # WARNING : experimental
    cov = cov[:3,:3]
    d = np.asarray([0.5, 0.5, np.deg2rad(30)])
    # +- 0.5m, 30deg. error
    p, _ = mvn.mvnun(
            -d, d,
            np.zeros_like(d),
            cov
            )
    # p = under current covariance estimtaes,
    # how likely will the pose fall
    # within defined error margin?
    return p

def ukf_filter(ukfs, est, obs, t, dt, obs_ar):
    n = len(est)
    m = len(obs)

    prob = np.zeros(shape=(n,m), dtype=np.float32)
    cost = np.ones(shape=(n,m), dtype=np.float32)

    i2k = est.keys()
    k2i = {i2k[i]:i for i in range(n)}

    # predict from dt
    for k in est.keys():
        ukfs[k].predict(dt, fx_args=t)
        est[k]._pose._data = ukfs[k].x.copy()
        # TODO : covariance -> probability?

    # assign
    for k in est.keys():
        for j, o in enumerate(obs):
            prob[k2i[k],j] = est[k].match(o)
            cost[k2i[k],j] = est[k].cost(o)

    #oob = [(est[i]._pose not in ar) for i in est.keys()]
    #cost[oob] += 9999
    #print cost

    # i.e. est[i] ~ obs[j]

    # update
    i_idx, j_idx = linear_sum_assignment(cost)
    #print i_idx, j_idx

    add_obs = np.ones(m)

    # TODO : track unmatched observations
    # TODO : clear low-probability estimations

    for (i,j) in zip(i_idx, j_idx):
        k = i2k[i]
        if est[k]._pose in obs_ar:
            if (prob[i,j] > P_MATCH):
                try:
                    ukfs[k].update(obs[j]._pose._data[:3])
                    est[k]._p0 = prob[i,j]
                    est[k]._t0 = t
                    add_obs[j] = False
                except Exception as e:
                    print e
                    print obs[j]._pose._data[:3]
            else:
                print k, prob[i,j]
                # TODO : "stray" estimate -- lower probability somehow.
                pass
        else:
            # no updates ...
            pass

        est[k]._pose._data = ukfs[k].x

    # clear ...
    good = []
    for k,u in ukfs.iteritems():
        #s = np.sqrt(np.sum(np.diag(u.P)[:2])) # variance-distance
        #if s < 2.0: # TODO : arbitrary threshold
        p = cov2p(u.P)
        if p > P_KEEP:
            good.append(k)
    ukfs = {k:ukfs[k] for k in good}
    est = {k:est[k] for k in good}

    new_j = np.where(add_obs)[0]
    if len(new_j) > 0:
        print new_j

    return ukfs, est, [obs[j] for j in new_j]

def main():
    # parameters
    n_targets = 14
    n_particles = 1 # particles per target
    dt = 0.1
    steps = (100.0 / dt)
    sigmas = np.asarray([S_X, S_Y, S_T, S_V, S_W])

    # initialization
    drone = Drone(Pose())
    targets = [SimpleParticle(Pose.random()) for i in range(n_targets)]
    mws = MerweScaledSigmaPoints(5,1e-3,2,-2,sqrt_method=sqrtm,subtract=ukf_residual)

    # render ...
    win_name = 'world'
    render = Renderer(win_name)
    def set_drone_pos(e,x,y,f,p):
        x,y = render._convert_inv(x,y)
        drone._pose.x=x
        drone._pose.y=y
    cv2.setMouseCallback(win_name,set_drone_pos)

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
    R = dt * np.diag(np.square([S_X, S_Y, S_T])) # measurement noise
    Q = dt * np.diag(np.square([0.01, 0.01, np.deg2rad(5), 0.01, 0.01]))
    # process noise, ~1cm / 5 deg. per second

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

    p_idx = np.max(particles.keys())
    t = 0
    while True:
        obs = CircularObservation(
                drone._pose.x,
                drone._pose.y,
                3.0 # TODO : arbitrary, fix
                )
        obs_rs = []
        for r in targets:
            if r._pose in obs:
                obs_r = r.clone()
                obs_r._pose = Pose(*add_noise(r._pose._data, s=dt*sigmas))
                #obs_r._pose = r._pose.clone()
                obs_rs.append(obs_r)

        probs = [cov2p(u.P) for u in ukfs.values()]
        k = render(
                drone._pose,
                [_t._pose for _t in targets],
                particles.values(),
                probs,
                t,
                delay=100
                )

        ukfs, particles, new = ukf_filter(ukfs, particles, obs_rs, t, dt, obs)

        # add particles ...
        # TODO : handle zero-len particles

        for _i in range(len(new)):
            i = p_idx
            pose = new[_i]._pose.clone()
            particles[i] = SimpleParticle(pose, p0=1.0, t0=t)
            ukfs[i] = UKF(**ukf_args)
            ukfs[i].Q = Q.copy() 
            ukfs[i].R = R.copy() 
            ukfs[i].x = pose._data.copy()
            ukfs[i].P = P.copy()
            p_idx += 1

        for _t in targets:
            _t.step(t,dt)

        if k == 27:
            break
        t += dt

if __name__ == "__main__":
    main()
