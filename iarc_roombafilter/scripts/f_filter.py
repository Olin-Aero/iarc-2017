import f_config as cfg
import numpy as np

def ukf_hx(x):
    return x[:3]

def ukf_fx(x0, dt, t, obs):
    x,y,th,v,w = x0
    if not obs:
        # not observable
        # make simulated predictions based on roomba model
        if t % cfg.INT_REVERSE < cfg.T_180:
            v = 0.0
            w = np.pi / cfg.T_180
            # 1.375
        elif t % cfg.INT_NOISE < cfg.T_NOISE:
            v = 0.0
            w = 0.0
            # assume constant w
            # only increase covariance.
        else:
            # go forward ...
            v = .33
            w = 0.0
    else:
        # respect current states
        # if under observation
        pass
    x += v * np.cos(th) * dt
    y += v * np.sin(th) * dt
    th += w * dt
    x = np.asarray([x,y,th,v,w])
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
