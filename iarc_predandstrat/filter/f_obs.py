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
    def __contains__(self, item):
        dx = item.x - self.x
        dy = item.y - self.y
        return (dx**2+dy**2 < self.r**2)

class ConicObservation(CircularObservation):
    def __init__(self, x, y, h, aov):
        self.x=x
        self.y=y
        self.r=self.h * np.tan(self.aov/2)
