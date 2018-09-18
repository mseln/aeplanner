import gp
import numpy as np
import matplotlib.pyplot as plt


hyperparam = gp.HyperParam(l = 0.3, sigma_f = 1, sigma_n = 0.1)

x = np.array([[0.4, 0.4], [-0.6, -0.6]])
y = np.array([0.719, -0.044])

xstar = np.empty([0, 2])
xi = np.arange(-1, 1, 0.01)
for i in xi:
    xstar = np.vstack((xstar, i*np.hstack([1,1])))

mu, sigma2 = gp.gp(y, x, xstar, hyperparam, gp.sqexpkernel)
sigma = np.sqrt(sigma2)

plt.plot(xi, mu)
plt.fill_between(xi, mu - 2*sigma, mu + 2*sigma, facecolor = '#D3D3D3', color = "#808080")
plt.show()
