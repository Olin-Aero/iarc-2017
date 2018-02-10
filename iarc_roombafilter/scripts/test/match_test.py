import rospkg
from geometry_msgs.msg import Twist, Pose2D, Point
rospack = rospkg.RosPack()

root = rospack.get_path('iarc_roombafilter')
import os
import sys
sys.path.append(os.path.join(root, 'scripts'))

import f_config as cfg
from f_particle import Particle

from matplotlib import pyplot as plt

import numpy as np

N = 1000 # number of particles to test
p0 = Particle([0,0,0,0,0])

poses = np.random.normal(scale=cfg.SIGMAS, size=[N,5])

cmap = plt.cm.get_cmap('plasma')
c = []
for pose in poses:
    p = Particle(pose)
    #c.append(p0.match(p))
    c.append(p0.cost(p))
print np.max(c)

names = ['x','y','h','v','w']
ax_xidx = 0
ax_yidx = 1

sc=plt.scatter(poses[:,ax_xidx], poses[:,ax_yidx], c=c, cmap=cmap)
#plt.scatter([0],[0],marker='+', c='r', lw=20)
plt.colorbar(sc)
plt.xlabel(names[ax_xidx])
plt.ylabel(names[ax_yidx])
#plt.colorbar('plasma')
plt.grid()
plt.show()
#p = [Particle(np.random. for _ in range(N)]
#p0.match(p)
