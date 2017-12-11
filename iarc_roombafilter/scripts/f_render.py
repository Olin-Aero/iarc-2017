import numpy as np
import cv2
from scipy.stats import mvn

class Renderer(object):
    def __init__(self, name='world', w=500, h=500, s=25):
        self._name = name
        self._w = w
        self._h = h
        self._s = s
        self._img = np.zeros(shape=(w,h,4), dtype=np.uint8)
        self._ff = cv2.FONT_HERSHEY_SIMPLEX
        self._fs = 0.5
        cv2.namedWindow(self._name)

    def _convert(self, x, y):
        x = self._w/2 + self._s*x
        y = self._h/2 + self._s*(-y)
        return (int(x),int(y))

    def _convert_inv(self, x, y):
        x = (float(x) - self._w/2) / self._s
        y = -(float(y) - self._h/2) / self._s
        return x, y

    def __call__(self,
            drone,
            roombas,
            particles,
            t,
            delay=10
            ):

        img = self._img.copy()
        img_p = self._img.copy()
        img_r = self._img.copy()

        # roombas
        for r in roombas:
            x, y = self._convert(r[0], r[1])
            vx, vy = 20*np.cos(r[2]), -20*np.sin(r[2])
            vx, vy = (int(e) for e in (vx,vy))
            cv2.circle(img,
                    (x,y),
                    10,
                    color=(255,255,0),
                    thickness=-1
                    )
            #cv2.circle(img,
            #        (x,y),
            #        int(self._s*0.5),
            #        color=(255,255,255),
            #        thickness=1
            #        )# error margin
            cv2.line(img,
                    (x,y),
                    (x+vx,y+vy),
                    (255,255,0),
                    4
                    )


        # particles
        for p in particles:
            x, y = self._convert(p._pose[0], p._pose[1])
            vx, vy = 10*np.cos(p._pose[2]), -10*np.sin(p._pose[2])
            vx, vy = (int(e) for e in (vx,vy))

            # covariance visualization ...
            # show 95% confidence interval
            cov = p.cov()
            w, h = self._s*abs(cov[0,0]), self._s*abs(cov[1,1])
            v = np.linalg.eigh(cov)[1]
            c_a = np.arctan2(-v[1, -1], v[0, -1])
            cv2.ellipse(img,
                    (x,y), #center
                    (int((1+w*2.44)/2.), int((1+h*2.44)/2)), #axes
                    np.rad2deg(c_a), #angle
                    0,
                    360,
                    (128,0,128),
                    -1
                    )

            cv2.circle(img,
                    (x,y),
                    3,
                    (255,0,0, int(255 * p.p())),
                    -1
                    )
            cv2.line(img,
                    (x,y),
                    (x+vx,y+vy),
                    (255,0,0),
                    2
                    )
            cv2.putText(img,
                    ('%.2f' % p.p()),
                    (x,y),
                    self._ff,
                    self._fs*0.5,
                    (255,255,255)
                    )

        # drone
        cv2.circle(img,
                 self._convert(drone[0], drone[1]),
                 self._s * 3,
                 (0,0,255),
                 1
                 )
        
        # meta-info
        cv2.putText(img, ('t:%.2f'%t), (0,20), self._ff, self._fs, (255,255,255))

        cv2.imshow(self._name, img)

        return cv2.waitKey(delay)
