import numpy as np

from collections import namedtuple
HyperParam = namedtuple("HyperParam", "l sigma_f sigma_n")

def sqexpkernel(x1, x2, hyperparam):
    n1 = x1.shape[0]
    n2 = x2.shape[0]
    K = np.empty([n1, n2])
    # print(n1, n2)

    for i in range(0,n2):
        # print("Hej")
        # print(x1)
        # print(x2[i,:])
        # print(x1 - x2[i,:])
        l = np.linalg.norm(x1 - x2[i,:], axis = 1)
        # print(l)
        # print(hyperparam.sigma_f**2 * np.exp(-0.5 * (l / hyperparam.l)**2))
        # print("BYE")
        K[:,i] = hyperparam.sigma_f**2 * np.exp(-0.5 * (l / hyperparam.l)**2)

    return K

def gp(y, x, xstar, hyperparam, kernel):
    if(y.shape[0] is 0 or x.shape[0] is 0):
        return (np.empty((0)), np.empty((0)))

    K     = kernel(x, x, hyperparam)            # K(x,x)
    kstar = kernel(x, xstar, hyperparam)        # K(x,x*)
    kss   = kernel(xstar, xstar, hyperparam)    # K(x*,x*)

    # Algorithm 2.1 from Rasmussen & Williams
    L = np.linalg.cholesky(K + hyperparam.sigma_n**2*np.identity(K.shape[0]))
    alpha = np.linalg.solve( np.transpose(L), np.linalg.solve(L, y))
    posterior_mean = np.matmul(np.transpose(kstar), alpha)

    v = np.linalg.solve(L, kstar)
    posterior_variance = np.diag(kss - np.matmul(np.transpose(v), v))

    return posterior_mean, posterior_variance
