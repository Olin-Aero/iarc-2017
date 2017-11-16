import numpy as np
import cv2

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
            x, y = self._convert(r.x, r.y)
            vx, vy = 20*np.cos(r.t), -20*np.sin(r.t)
            vx, vy = (int(e) for e in (vx,vy))
            cv2.circle(img,
                    (x,y),
                    10,
                    color=(255,255,0),
                    thickness=-1
                    )
            cv2.line(img,
                    (x,y),
                    (x+vx,y+vy),
                    (255,255,0),
                    4
                    )

        # particles
        for p in particles:
            #print 'p', p.p(t)
            x, y = self._convert(p._pose.x, p._pose.y)
            vx, vy = 10*np.cos(p._pose.t), -10*np.sin(p._pose.t)
            vx, vy = (int(e) for e in (vx,vy))
            cv2.circle(img,
                    (x,y),
                    5,
                    (255,0,0, int(255 * p.p(t))),
                    -1
                    )
            cv2.line(img,
                    (x,y),
                    (x+vx,y+vy),
                    (255,0,0),
                    2
                    )
            cv2.putText(img,
                    ('%.2f' % p.p(t)),
                    (x,y),
                    self._ff,
                    self._fs*0.5,
                    (255,255,255)
                    )


        # drone
        cv2.circle(img,
                 self._convert(drone.x, drone.y),
                 self._s * 5,
                 (0,0,255),
                 1
                 )
        
        # meta-info
        cv2.putText(img, ('t:%.2f'%t), (0,20), self._ff, self._fs, (255,255,255))

        cv2.imshow(self._name, img)

        return cv2.waitKey(delay)
