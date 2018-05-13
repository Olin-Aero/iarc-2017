import numpy as np

from f_obs import *
from f_render import *
from f_filter import *
from f_particle import *

def add_noise(x, s):
    """
    Adds noise sampled from truncated-normal distribution.
    i.e. the noise will be within 2 standard deviations.

    Parameters:
    x : original data
    s : standard deviation, scalar or vector-like, same length as data
    """
    z = np.random.normal(scale=s)

    # make sure 2 standard deviations
    msk = (np.abs(z) >= 2*s)
    while np.any(msk):
        z[msk] = np.random.normal(scale=s[msk])
        msk = (np.abs(z) >= 2*s)
    return x + z

def vqv(v0, v1, norm=False, eps=1e-6):
    """
    Quaternion from vector to vector.
    Format : wxyz
    """
    q = np.zeros(4, dtype=np.float32)
    if not norm:
        v0 = v0 / np.linalg.norm(v0)
        v1 = v1 / np.linalg.norm(v1)

    if np.dot(v0,v1) > (1-eps):
        q = np.asarray([1,0,0,0], dtype=np.float32)
    elif np.dot(v0,v1) < -(1-eps):
        q = np.asarray([0,1,0,0], dtype=np.float32)
    else:
        q[0] = 1 + np.dot(v0,v1) # angle
        q[1:] = np.cross(v0,v1) # axis
    q /= np.linalg.norm(q)
    return q

def qxq2(quaternion1, quaternion0):
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return np.array((
         x1*w0 + y1*z0 - z1*y0 + w1*x0,
        -x1*z0 + y1*w0 + z1*x0 + w1*y0,
         x1*y0 - y1*x0 + z1*w0 + w1*z0,
        -x1*x0 - y1*y0 - z1*z0 + w1*w0), dtype=np.float64)

def qxq(q1, q2):
    """ quaternion format : wxyz """
    res = [q2[0]*q1[0]-q2[1]*q1[1]-q2[2]*q1[2]-q2[3]*q1[3],
            q2[0]*q1[1]+q2[1]*q1[0]-q2[2]*q1[3]+q2[3]*q1[2],
            q2[0]*q1[2]+q2[1]*q1[3]+q2[2]*q1[0]-q2[3]*q1[1],
            q2[0]*q1[3]-q2[1]*q1[2]+q2[2]*q1[1]+q2[3]*q1[0]]
    return res

def qxv(q, v, norm=True):
    """
    Multiply vector by a quaternion.
    (Effectively, a rotation)

    q : wxyz
    v : xyz
    """
    if not norm:
        s = np.linalg.norm(v)
        v = v / s # make unit
    q_v = [0] + list(v)
    q_c = [q[0], -1*q[1], -1*q[2], -1*q[3]] #w,-x,-y,-z
    v = qxq(qxq(q,q_v),q_c)[1:]
    if not norm:
        v = np.multiply(v,s)
    return v

def observability(K, w, h, q, t, inv=False, ray=False):
    """
    K : 3x3 camera projection matrix; K^-1 if inv==True
    w : width of camera image
    h : height of camera image
    q : rotation quaternion from camera frame to world frame
    t : camera position 
    """
    # K = 3x3 camera projection 
    if not inv:
        K = np.linalg.inv(K)
    vs = np.transpose([(0,0,1), (w,0,1), (w,h,1), (0,h,1)])
    vs = K.dot(vs) # rays in camera coordinates

    # rotate to camera coordinates
    vs = [qxv(q, v, norm=False) for v in vs.T]
    vs = [v/np.linalg.norm(v) for v in vs]
    vs0 = np.copy(vs)

    #l = [vqv(v0,v1) for (v0,v1) in zip(vs, vsp)]

    ps = [-t[2] / v[2] for v in vs]
    vs = [np.add(t, np.multiply(p, v)) for (p,v) in zip(ps,vs)]

    if ray:
        return vs, vs0
    else:
        return vs

    # now compute intersection to ground plane ...

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

if __name__ == "__main__":
    from matplotlib import pyplot as plt
    #from matplotlib.patches import Polygon
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection as Polygon
    from mpl_toolkits.mplot3d import Axes3D

    K = [[374.6706071,   0.       , 320.5      ],
         [  0.       , 374.6706071, 180.5      ],
         [  0.       ,   0.       ,   1.       ]]
    w = 640
    h = 360
    t = [2, 5, 3]
    #v0 = [0.5,0.5,-1.0]
    v0 = [0.3, 0.1, -1.0]
    v0 = np.divide(v0, np.linalg.norm(v0))

    q = vqv(v0,[0,0,1]) # viewing a bit right-up.

    # camera. in world
    cx = qxv(q,[1,0,0])
    cy = qxv(q,[0,1,0])
    cz = qxv(q,[0,0,1])

    c_ax_x = np.asarray([t, np.add(t, cx*1)])
    c_ax_y = np.asarray([t, np.add(t, cy*1)])
    c_ax_z = np.asarray([t, np.add(t, cz*1)])

    #q = vqv([0.,0.,1.0],[0,0,1]) # viewing a bit right-up.

    ar, rays = observability(K,w,h,q,t, ray=True)

    rays0 = np.asarray([t, np.add(t, 5*rays[0])])
    rays1 = np.asarray([t, np.add(t, 5*rays[1])])
    rays2 = np.asarray([t, np.add(t, 5*rays[2])])
    rays3 = np.asarray([t, np.add(t, 5*rays[3])])

    ar = np.asarray(ar)
    rays = np.asarray(rays)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_aspect('equal')

    # show camera ...
    ax.plot(c_ax_x[:,0], c_ax_x[:,1], c_ax_x[:,2], 'r')
    ax.plot(c_ax_y[:,0], c_ax_y[:,1], c_ax_y[:,2], 'g')
    ax.plot(c_ax_z[:,0], c_ax_z[:,1], c_ax_z[:,2], 'b')

    # show rays ...
    ax.plot(rays0[:,0], rays0[:,1], rays0[:,2], 'r')
    ax.plot(rays1[:,0], rays1[:,1], rays1[:,2], 'g')
    ax.plot(rays2[:,0], rays2[:,1], rays2[:,2], 'b')
    ax.plot(rays3[:,0], rays3[:,1], rays3[:,2], 'b')

    # show projections ...
    p_ar = Polygon([ar])
    ax.add_collection3d(p_ar)
    ax.plot(ar[:,0], ar[:,1], ar[:,2], 'k')
    set_axes_equal(ax)


    plt.show()
    #ax.scatter(v0[:,0], v0[:,1], v0[:,2], c='r')
    #ax.scatter(v1[:,0], v1[:,1], v1[:,2], c='b')
    #ax.set_xlabel('x')
    #ax.set_ylabel('y')
    #ax.set_zlabel('z')
    #plt.show()
