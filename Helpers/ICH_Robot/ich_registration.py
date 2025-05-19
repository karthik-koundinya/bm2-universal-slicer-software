from __future__ import annotations

from typing import Tuple, Optional
import numpy as np

__all__ = ["register_points"]


def register_points(X: np.ndarray, Y: np.ndarray, w: np.Optional[ndarray] = None, n_t: Optional[int] = None) -> Tuple[np.ndarray, np.ndarray, float]:
    K, N = X.shape
    if Y.shape != (K, N):
        raise ValueError("X and Y must have same shape")

    if w is None:
        w = np.ones(N)
    w = w / w.sum()

    if n_t is None:
        n_t = N
    n_t = max(0, min(n_t, N))

    Xbar = (X[:, :n_t] * w[:n_t]).sum(axis=1, keepdims=True)
    Ybar = (Y[:, :n_t] * w[:n_t]).sum(axis=1, keepdims=True)

    Xc = X - Xbar
    Yc = Y - Ybar

    S = Xc * w
    H = S @ Yc.T
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:  
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = Ybar - R @ Xbar

    diff = R @ X + t - Y
    FRE = np.linalg.norm(diff, axis=0).mean()
    return R, t, FRE