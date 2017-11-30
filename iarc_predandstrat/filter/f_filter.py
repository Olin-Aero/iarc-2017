import numpy as np

def ukf_hx(x):
    return x[:3]

def ukf_fx(x0, dt, t):
    # TODO : support time-based transition?
    # or handle it outside of ukf by manipulating state
    # I think fx_args in predict() can work?

    # x-y-th-v-w
    x,y,th,v,w = x0
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


