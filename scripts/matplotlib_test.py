# from pylab import *
# from mpl_toolkits.mplot3d import Axes3D
# from matplotlib import pyplot as plt

# fig = figure()
# ax = Axes3D(fig)
# X = np.arange(-4, 4, 0.25)
# Y = np.arange(-4, 4, 0.25)
# X, Y = np.meshgrid(X, Y)
# R = np.sqrt(X**2 + Y**2)
# Z = np.sin(R)

# ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap='hot')
#show()

# ax = plt.figure().add_subplot(projection='3d')

# ax.scatter([0, 1, 2], [1, 3, 5], [30, 50, 70])

# ax.set_xticks([0.25, 0.75, 1.25, 1.75], minor=True)
# ax.set_xticklabels(['a', 'b', 'c', 'd'], minor=True)

# ax.set_yticks([1.5, 2.5, 3.5, 4.5], minor=True)
# ax.set_yticklabels(['A', 'B', 'C', 'D'], minor=True)

# ax.set_zticks([35, 45, 55, 65], minor=True)
# ax.set_zticklabels([r'$\alpha$', r'$\beta$', r'$\delta$', r'$\gamma$'],minor=True)

# ax.tick_params(which='major', color='C0', labelcolor='C0', width=5)
# ax.tick_params(which='minor', color='C1', labelcolor='C1', width=3)


import matplotlib.pyplot as plt
import numpy as np

# Fixing random state for reproducibility
np.random.seed(19680801)


def randrange(n, vmin, vmax):
    """
    Helper function to make an array of random numbers having shape (n, )
    with each number distributed Uniform(vmin, vmax).
    """
    return (vmax - vmin)*np.random.rand(n) + vmin

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

n = 100

# For each set of style and range settings, plot n random points in the box
# defined by x in [23, 32], y in [0, 100], z in [zlow, zhigh].
for m, zlow, zhigh in [('o', -50, -25), ('^', -30, -5)]:
    xs = randrange(n, 23, 32)
    ys = randrange(n, 0, 100)
    zs = randrange(n, zlow, zhigh)
    ax.scatter(xs, ys, zs, marker=m)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()