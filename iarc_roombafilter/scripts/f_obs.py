"""
Observation Data.
Really feels like overkill.
"""
import numpy as np

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
    def __contains__(self, p):
        """
        Check for existence within observation area.

        Parameters:

        p : np.array(shape=5, dtype=np.float32)
        Formatted as [x,y,t,v,w]
        """
        dx = p[0] - self.x
        dy = p[1] - self.y
        return (dx**2+dy**2 < self.r**2)

class ConicObservation(CircularObservation):
    def __init__(self, x, y, h, aov):
        self.x=x
        self.y=y
        self.r=self.h * np.tan(self.aov/2)
