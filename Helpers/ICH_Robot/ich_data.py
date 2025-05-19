from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Union, Tuple

AxisVec = List[float]

# ──────────────────────────────────────────────────────────────────────────────
# Global design parameters 
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class ICHDesignParameters:
    L_A: float = 100.0
    L_B: float = 135.0
    kappa: float = 1.0 / 27.16
    s_max: float = 35.0

    c1y: float = -0.014
    c2y: float = -0.0373
    c3y: float = 0.0

    c1z: float = -0.0081
    c2z: float = 1.0831
    c3z: float = 0.0

    c1phi: float = -24.0 / s_max
    c2phi: float = 24.0

    frame_point_z_offset: float = -27.58
    frame_point1: list[float] = field(default_factory=lambda: [-55.0, 0.0, -27.58])
    frame_point2: list[float] = field(default_factory=lambda: [0.0, -35.0, -27.58])
    frame_point3: list[float] = field(default_factory=lambda: [15.0, 15.0, -27.58])

    R: Any = field(default_factory=lambda: [[1.0, 0.0, 0.0],
                                            [0.0, 1.0, 0.0],
                                            [0.0, 0.0, 1.0]])
    t: Any = field(default_factory=lambda: [0.0, 0.0, 0.0])

    R0_aim: Any = field(default_factory=lambda: [[1.0, 0.0, 0.0],
                                                 [0.0, 1.0, 0.0],
                                                 [0.0, 0.0, 1.0]])
    t_aim: Any = field(default_factory=lambda: [0.0, 0.0, 0.0])

    R0_robot: Any = field(default_factory=lambda: [[1.0, 0.0, 0.0],
                                                   [0.0, 1.0, 0.0],
                                                   [0.0, 0.0, 1.0]])
    t_robot: Any = field(default_factory=lambda: [0.0, 0.0, 0.0])

    origin_ras: Any = field(default_factory=lambda: [0.0, 0.0, 0.0])

    scales: Dict[str, float] = field(default_factory=lambda: {
        "A": (25.4 / 32) * (1 / 512) / 4,   # mm / pulse  (A‑axis)
        "B": (25.4 / 32) * (1 / 512) / 4,   # mm / pulse  (B‑axis)
        "C": 360.0 / 512 / 4,               # deg / pulse (C‑axis)
    })

    ctr_insertion_offset: float = 2.0


# ──────────────────────────────────────────────────────────────────────────────
# Axis‑runtime status
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class AxisStatus:
    controller_name: str
    name: str
    scale: float

    # live values
    cur_pulses: float = 0.0
    target_pulses: float = 0.0

    # motion & control params
    motor_configuration: int = 1
    encoder_configuration: int = 0
    acceleration: int = 1_000_000
    deceleration: int = 1_000_000
    ls_deceleration: int = 900_000
    kp: int = 0
    kd: int = 0
    ki: int = 0
    slew_speed: int = 1_000_000
    torque_limit: float = 9.999

    # soft limits & thresholds
    lower_limit: float = 0.0
    upper_limit: float = 0.0
    err_thresh: float = 0.0

    limit_configuration: int = 1
    homing_configuration: int = 1

    # limit‑switch bits
    lr_status: int = 0
    lf_status: int = 0
    hm_status: int = 0

    # homing speeds (C‑axis only)
    homing_jg: int = 0
    homing_hv: int = 0

    # UI helpers
    is_target_valid: bool = True
    is_home_limit_inverted: bool = False

    # helper
    def within_limits(self, pulses: float) -> bool:
        return self.lower_limit <= pulses <= self.upper_limit


class ICHRobotConfiguration:
    @staticmethod
    def default() -> List[AxisStatus]:
        p = ICHDesignParameters()
        s = p.scales

        axis_a = AxisStatus(
            controller_name="A",
            name="A:Insert Outer",
            scale=s["A"],
            kp=100,
            kd=4000,
            lower_limit=0 / s["A"],
            upper_limit=100 / s["A"],
            err_thresh=1 / s["A"],        
        )

        axis_b = AxisStatus(
            controller_name="B",
            name="B:Insert Inner",
            scale=s["B"],
            lower_limit=0 / s["B"],
            upper_limit=135 / s["B"],
            err_thresh=1 / s["B"],        
        )

        axis_c = AxisStatus(
            controller_name="C",
            name="C:Rotate Inner",
            scale=s["C"],
            lower_limit=-360 / s["C"],
            upper_limit=+360 / s["C"],
            err_thresh=5 / s["C"],        
            homing_jg=300,
            homing_hv=100,
        )

        return [axis_a, axis_b, axis_c]