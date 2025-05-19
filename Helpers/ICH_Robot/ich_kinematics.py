from __future__ import annotations

from math import sin, cos, atan2, sqrt, asin, pi, degrees, radians
from typing import List, Union, Tuple
import numpy as np

from .ich_data import ICHDesignParameters

__all__ = [
    "ICHKinematics",
]


class ICHKinematics: 
    def __init__(self, params: ICHDesignParameters) -> None:
        self.p = params

    # ------------------------------------------------------------------
    # Forward kinematics
    # ------------------------------------------------------------------
    def fk(self, joint: Union[List[float], Tuple[float, float, float]]) -> np.ndarray:
        """Return 4×4 pose matrix given *joint* = [I_A (mm), I_B (mm), alpha (rad)]."""
        I_A, I_B, alpha = joint
        s = max(0.0, I_B - I_A)

        kappa = self.p.kappa
        c1y, c2y, c3y = self.p.c1y, self.p.c2y, self.p.c3y
        c1z, c2z, c3z = self.p.c1z, self.p.c2z, self.p.c3z

        rot = np.array(
            [
                [cos(alpha), -cos(kappa * s) * sin(alpha), sin(kappa * s) * sin(alpha)],
                [sin(alpha), cos(kappa * s) * cos(alpha), -sin(kappa * s) * cos(alpha)],
                [0.0, sin(kappa * s), cos(kappa * s)],
            ]
        )

        tip = np.array(
            [
                -sin(alpha) * (c1y * s**2 + c2y * s + c3y),
                cos(alpha) * (c1y * s**2 + c2y * s + c3y),
                c1z * s**2 + c2z * s + I_A + c3z,
            ]
        )

        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = tip
        return T

    # ------------------------------------------------------------------
    # Inverse kinematics
    # ------------------------------------------------------------------
    def ik(self, xyz: Union[List[float], Tuple[float, float, float]], current_joint: List[float]) -> List[float]:
        x, y, z = xyz
        p = self.p

        alpha_unch = atan2(x, -y)

        A = p.c1y
        B = p.c2y
        C = p.c3y - y * cos(alpha_unch) + x * sin(alpha_unch)
        disc = B**2 - 4 * A * C
        if disc < 0:
            raise ValueError("Target unreachable – discriminant < 0")
        s = (-B - sqrt(disc)) / (2 * A)  

        I_A = z - (p.c1z * s**2 + p.c2z * s + p.c3z)
        I_B = I_A + s

        s_current = current_joint[1] - current_joint[0]
        alpha_current = radians(current_joint[2])  

        if s_current < s:
            alpha = alpha_unch - (p.c1phi * s + p.c2phi) * pi / 180.0 * np.sign(alpha_current - alpha_unch)
        else:
            alpha = alpha_unch - (p.c1phi * s_current + p.c2phi) * pi / 180.0 * np.sign(alpha_current - alpha_unch)

        return [I_A, I_B, alpha]

    # ------------------------------------------------------------------
    # Helper
    # ------------------------------------------------------------------
    def aiming2ctr(self, q1: float, q2: float, q3_deg: float, q4_deg: float) -> np.ndarray:
        q3 = radians(q3_deg)
        q4 = radians(q4_deg)

        T = (
            self._T(q1, q2, 0)
            @ self._rotz(q3, True)
            @ self._roty(q4, True)
            @ self._rotz(pi / 2, True)
            @ self._T(0, 0, self.p.ctr_insertion_offset)
        )
        return T

    # ------------------------------------------------------------------
    # Small helpers (private)
    # ------------------------------------------------------------------
    @staticmethod
    def _T(x: float, y: float, z: float) -> np.ndarray:
        return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

    @staticmethod
    def _rotx(t: float, is4: bool = False) -> np.ndarray:
        c, s = cos(t), sin(t)
        if is4:
            return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    @staticmethod
    def _roty(t: float, is4: bool = False) -> np.ndarray:
        c, s = cos(t), sin(t)
        if is4:
            return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    @staticmethod
    def _rotz(t: float, is4: bool = False) -> np.ndarray:
        c, s = cos(t), sin(t)
        if is4:
            return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])