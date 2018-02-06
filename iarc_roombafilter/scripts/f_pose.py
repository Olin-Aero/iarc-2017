import numpy as np

## Pose
#class Pose(object):
#    def __init__(self,
#            x=0,y=0,t=0,
#            v=0,w=0):
#        self._data = np.asarray([x,y,t,v,w], dtype=np.float32)
#    def __repr__(self):
#        r = {
#                'x':self.x,
#                'y':self.y,
#                't':self.t,
#                'v':self.v,
#                'w':self.w
#                }
#        return str(r)
#    def __str__(self):
#        return repr(self)
#    @property
#    def x(self):
#        return self._data[0]
#    @x.setter
#    def x(self,_x):
#        self._data[0]=_x
#    @property
#    def y(self):
#        return self._data[1]
#    @y.setter
#    def y(self,_y):
#        self._data[1]=_y
#    @property
#    def t(self):
#        return self._data[2]
#    @t.setter
#    def t(self,_t):
#        self._data[2]=_t
#    @property
#    def v(self):
#        return self._data[3]
#    @v.setter
#    def v(self,_v):
#        self._data[3]=_v
#    @property
#    def w(self):
#        return self._data[4]
#    @w.setter
#    def w(self,_w):
#        self._data[4]=_w
#    @staticmethod
#    def random():
#        # TODO : kind-of arbitrary numbers here
#        x = np.random.uniform(-10.0, 10.0)
#        y = np.random.uniform(-10.0, 10.0)
#        t = np.random.uniform(-np.pi, np.pi)
#        v = np.abs(np.random.normal(0.27, 0.33))
#        w = np.random.choice([-1,1]) * np.random.normal(0.8, 1.2)
#        return Pose(x,y,t,v,w)
#    def step(self, dt):
#        self.x += (self.v * np.cos(self.t) * dt)
#        self.y += (self.v * np.sin(self.t) * dt)
#        self.t += (self.w * dt)
#    def clone(self):
#        return Pose(*self._data)
