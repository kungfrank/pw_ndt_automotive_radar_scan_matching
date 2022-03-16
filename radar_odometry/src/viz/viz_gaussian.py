import numpy as np
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

# Our 2-dimensional distribution will be over variables X and Y
N = 600
X = np.linspace(-3, 3, N)
Y = np.linspace(-3, 3, N)
X, Y = np.meshgrid(X, Y)

# Mean vector and covariance matrix
mu = np.array([0., 0.])
Sigma = np.array([[ 0.155 , -0.145], [-0.145,  0.155]])



# Pack X and Y into a single 3-dimensional array
pos = np.empty(X.shape + (2,))
pos[:, :, 0] = X
pos[:, :, 1] = Y

def multivariate_gaussian(pos, mu, Sigma):
    """Return the multivariate Gaussian distribution on array pos.

    pos is an array constructed by packing the meshed arrays of variables
    x_1, x_2, x_3, ..., x_k into its _last_ dimension.

    """

    n = mu.shape[0]
    Sigma_det = np.linalg.det(Sigma)
    Sigma_inv = np.linalg.inv(Sigma)
    N = np.sqrt((2*np.pi)**n * Sigma_det)
    # This einsum call calculates (x-mu)T.Sigma-1.(x-mu) in a vectorized
    # way across all the input variables.
    fac = np.einsum('...k,kl,...l->...', pos-mu, Sigma_inv, pos-mu)

    return np.exp(-fac / 2) / N
'''
# The distribution on the variables X, Y packed into pos.
Z = multivariate_gaussian(pos, mu, Sigma)

# Create a surface plot and projected filled contour plot under it.
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_surface(X, Y, Z, rstride=3, cstride=3, linewidth=1, antialiased=True,
                cmap=cm.viridis)

cset = ax.contourf(X, Y, Z, zdir='z', offset=-0.15, cmap=cm.viridis)

# Adjust the limits, ticks and view angle
ax.set_zlim(-0.15,0.2)
ax.set_zticks(np.linspace(0,0.2,5))
ax.view_init(27, -21)
'''
#Z = multivariate_gaussian(pos, mu, Sigma)
#plt.contourf(X, Y, Z*10, 30, alpha=1)

#rv = multivariate_normal([0.5, -0.2], [[2.0, 0.3], [0.3, 0.5]])

rv = multivariate_normal([0.6, 0.0], [[ 0.07 , -0.2], [-0.2,  5]]) # dense ex
#rv = multivariate_normal([0.08, 0.0], [[ 0.05 , -0.1], [-0.1,  5.3]]) # sparse ex
plt.contourf(X, Y, rv.pdf(pos), 70, cmap='gray')

#C = plt.contour(X, Y, Z*10, 30, colors='black', linewidth=.5)
#plt.clabel(C, inline=True, fontsize=10)

plt.xticks(())
plt.yticks(())

#plt.savefig("/home/joinet/test.png", bbox_inches='tight')
plt.show()



