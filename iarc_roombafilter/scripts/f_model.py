"""
Roomba Motion Models for the filter to apply.
"""

from abc import ABCMeta, abstractmethod
import numpy as np

class BaseRoombaModel(object):
    __metaclass__ = ABCMeta
    def __init__(self):
        pass
    @abstractmethod
    def __call__(self, s, t, dt):
        return NotImplementedError("BaseRoombaModel cannot be applied!")

class StaticRoombaModel(BaseRoombaModel):
    """ Stays in place """
    def __init__(self):
        super(StaticRoombaModel, self).__init__()
    def __call__(self, s, t, dt):
        return s

class SimpleRoombaModel(BaseRoombaModel):
    """ Preserves velocity information """
    def __init__(self):
        super(SimpleRoombaModel,self).__init__()
    def __call__(self, s, t, dt):
        x,y,h,v,w = s
        x += v * np.cos(h) * dt
        y += v * np.sin(h) * dt
        h += w * dt
        return [x,y,h,v,w]

class TargetRoombaModel(SimpleRoombaModel):
    """ Target Roomba """
    def __init__(self, opts):
        self._opts = opts
        super(TargetRoombaModel, self).__init__()
    def __call__(self, s, t, dt):
        opts = self._opts
        if t % opts['t_r'] < opts['d_180']:
            v = 0.0
            w = np.pi / opts['d_180']
        elif t % opts['t_n'] < opts['d_n']:
            # here, angular variance should be increased.
            v = 0.0
            w = 0.0
        else:
            v = opts['v']
            w = 0.0
        s = [s[0], s[1], s[2], v, w]
        return SimpleRoombaModel.__call__(self, s,t,dt)

class ObstacleRoombaModel(SimpleRoombaModel):
    """ Obstacle Roomba """
    def __init__(self, opts):
        self._opts = opts
        super(ObstacleRoombaModel, self).__init__()
    def __call__(self, s, t, dt):
        s = [s[0], s[1], s[2], self._opts['v'], self._opts['w']]
        return SimpleRoombaModel.__call__(self, s,t,dt)



def main():
    from matplotlib import pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.animation import TimedAnimation

    class MultiCometAnimation(TimedAnimation):
        def __init__(self, data, legend, fig=None, **kwargs):
            # fig setup ... 
            if not fig:
                fig = plt.figure()
            ax = fig.add_subplot(1,1,1)

            # data setup ...
            self._data = data
            n, m = np.shape(data)[:2]
            self._n = n
            self._m = m
            
            xmin = np.min(data[:,:,0])
            xmax = np.max(data[:,:,0])
            xmid = (xmax+xmin)/2.
            ymin = np.min(data[:,:,1])
            ymax = np.max(data[:,:,1])
            ymid = (ymax+ymin)/2.

            s = max(xmax-xmin, ymax-ymin)
            # circle radius
            r = 0.01 * s

            # plots setup ...
            self._ls = []
            self._cs = []
            for i in range(m):
                col = np.random.uniform(size=3)
                cir = plt.Circle((0,0), r, fc=col)
                ln = Line2D([],[], color=col)
                self._cs.append(cir)
                self._ls.append(ln)
                ax.add_patch(cir)
                ax.add_line(ln)

            ax.set_aspect('equal')
            ax.set_xlim(xmid-s/2., xmid+s/2.)
            ax.set_ylim(ymid-s/2., ymid+s/2.)
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.legend(legend)

            TimedAnimation.__init__(self, fig, **kwargs)

        def _draw_frame(self, k):
            for i in range(self._m):
                x,y = self._data[:k, i, :2].T
                self._ls[i].set_data(x,y)
                self._cs[i].center = x[-1], y[-1]
            self._drawn_artists = self._ls + self._cs

        def new_frame_seq(self):
            return iter(range(1, self._n))

        def _init_draw(self):
            pass

    # create roomba models
    r1 = StaticRoombaModel()
    r2 = SimpleRoombaModel()
    r3 = TargetRoombaModel({
        # speed configurations
        'v'    : 0.33,
        'w'    : 0.066,
        # intervals
        't_n'  : 5.0, # noise interval
        't_r'  : 20.0, # reversal interval
        # durations
        'd_180': 2.15,
        'd_45' : 2.15/4,
        'd_n'  : 0.85
        })

    r4 = ObstacleRoombaModel({
        'v'    : 0.33,
        'w'    : 0.066
        })

    # initial testing conditions
    x = np.random.uniform(-10, 10)
    y = np.random.uniform(-10, 10)
    h = np.random.uniform(-np.pi, np.pi)
    v = np.random.uniform(0.2, 0.8)
    w = np.random.uniform(0.5, 2.5)
    s = [x,y,h,v,w]
    t_start = 0.0
    t_final = 200.0
    dt = 0.05

    #print r0(s, t, dt)
    #print r1(s, t, dt)
    #print r2(s, t, dt)
    #print r3(s, t, dt)

    # initialize
    t = t_start
    s1 = np.copy(s)
    s2 = np.copy(s)
    s3 = np.copy(s)
    s4 = np.copy(s)

    # storage 
    ts = []
    s1s = []
    s2s = []
    s3s = []
    s4s = []

    # simulate
    while t < t_final:
        ts.append(t)
        s1s.append(np.copy(s1))
        s2s.append(np.copy(s2))
        s3s.append(np.copy(s3))
        s4s.append(np.copy(s4))
        s1 = r1(s1, t, dt)
        s2 = r2(s2, t, dt)
        s3 = r3(s3, t, dt)
        s4 = r4(s4, t, dt)
        t += dt

    s1s = np.asarray(s1s)
    s2s = np.asarray(s2s)
    s3s = np.asarray(s3s)
    s4s = np.asarray(s4s)
    data = np.stack([s1s,s2s,s3s,s4s], axis=1)

    legend = ['static', 'simple', 'target', 'obstacle']
    ani = MultiCometAnimation(
            data, legend,
            interval=1000/120., blit=True, repeat=True
            )
    plt.show()

    #plt.plot(s1s[:,0], s1s[:,1])
    #plt.plot(s2s[:,0], s2s[:,1])
    #plt.plot(s3s[:,0], s3s[:,1])
    #plt.plot(s4s[:,0], s4s[:,1])
    #plt.legend(['static','simple','target','obstacle'])
    #plt.show()

if __name__ == "__main__":
    main()
