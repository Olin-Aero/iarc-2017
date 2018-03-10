"""
Generic Observation-Area.
"""

import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

point = Point(0.5, 0.5)
polygon = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
print(polygon.contains(point))

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
        super(ConicObservation, self).__init__(x, y, h * np.tan(aov/2))

class PolygonObservation(Observation):
    def __init__(self, pts):
        super(PolygonObservation, self).__init__()
        self._poly = Polygon(pts)
    def __contains__(self, p):
        return self._poly.contains(Point(p))
