import numpy as np


def quintic(u):
    """Position and first two derivatives with respect to normalized time."""
    u = np.asarray(u, dtype=float)
    return (u**3 * (10 - 15*u + 6*u*u),
            30*u*u*(1-u)**2,
            60*u*(1-u)*(1-2*u))
