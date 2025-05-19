from __future__ import annotations

import os
import textwrap

import math
import numpy as np
import qt 
import logging
from typing import List, Tuple, Any, Optional

from ..base_interfaces import BaseRobot, BaseController
from .ich_data import ICHDesignParameters, ICHRobotConfiguration, AxisStatus
from .ich_kinematics import ICHKinematics

_LOG = logging.getLogger(__name__)

AxisVec = List[float]
SeqStep = Tuple[int, float]

class ICHRobot(BaseRobot):
    def __init__(self, controller: BaseController) -> None:
        super().__init__(controller)

        # design & config
        self.params = ICHDesignParameters()
        self.axes: List[AxisStatus] = ICHRobotConfiguration.default()
        self.kin = ICHKinematics(self.params)

        # runtime queues & flags
        self._motion_seq: List[SeqStep] = []
        self._homing_seq: List[Any] = []
        self._halt_flag: bool = False

        # timers
        self._timer_motion = qt.QTimer()
        self._timer_motion.timeout.connect(self._step_motion_sequence)
        self._timer_motion.start(100)

        self._timer_homing = qt.QTimer()
        self._timer_homing.timeout.connect(self._step_homing_sequence)
        self._timer_homing.start(100)

    def initial_config(self) -> None:
        c = self.controller
        if not c.is_connected:
            raise RuntimeError("Controller not connected")

        # echo, stop, servo-on
        c.send("EO 1")
        c.send("ST")
        c.send("SH")

        # acceleration/decel
        ac = ", ".join(str(ax.acceleration) for ax in self.axes)
        dc = ", ".join(str(ax.deceleration) for ax in self.axes)
        sd = ", ".join(str(ax.ls_deceleration) for ax in self.axes)
        c.send(f"AC {ac}")
        c.send(f"DC {dc}")
        c.send(f"SD {sd}")

        # PID gains
        kp = ", ".join(str(ax.kp) for ax in self.axes)
        kd = ", ".join(str(ax.kd) for ax in self.axes)
        ki = ", ".join(str(ax.ki) for ax in self.axes)
        c.send(f"KP {kp}")
        c.send(f"KD {kd}")
        c.send(f"KI {ki}")

        # torque limit
        c.send(f"TL {self.axes[0].torque_limit}")

        # encoder/motor config
        ce = ", ".join(str(ax.encoder_configuration) for ax in self.axes)
        mt = ", ".join(str(ax.motor_configuration) for ax in self.axes)
        sp = ", ".join(str(ax.slew_speed) for ax in self.axes)
        c.send(f"CE {ce}")
        c.send(f"MT {mt}")
        c.send(f"SP {sp}")

        # correct CN using stored params
        first = self.axes[0]
        c.send(f"CN {first.limit_configuration}, {first.homing_configuration}")

        for idx in [2, 0, 1]:
            c.send(f"LD{self.axes[idx].controller_name}=3")

        self.update_status()
        for ax in self.axes:
            ax.target_pulses = ax.cur_pulses
            c.send(f"OF{ax.controller_name}=0")

        _LOG.info("Initial configuration completed.")

    # ──────────────────────────────────────────────────────────────────────────
    # Public motion primitives
    # ──────────────────────────────────────────────────────────────────────────
    def move_single_axis(self, axis_idx: int, target_pulses: int) -> None:
        """Queue a single-axis move (for Set button)."""
        self._halt_flag = False
        self.controller.send("SH")
        self._motion_seq = [(axis_idx, float(target_pulses))]

    def insert(self, *, distance_mm: float) -> None:
        """Simple jog/insert by delta mm on A and B."""
        cur = self._current_joint_vector()
        eng = [cur[0] + distance_mm, cur[1] + distance_mm, cur[2]]
        self._push_axis_vector(eng, interpolate=True)

    def move_all_axes(self) -> None:
        """Move all axes to the previously set _target_robot_xyz."""
        eng = self.kin.ik(self._target_robot_xyz, self._current_joint_vector())
        eng[2] = math.degrees(eng[2])
        self._push_axis_vector(eng, interpolate=False)

    def home(self, *, pack_and_go: bool = False) -> None:
        """Homing sequence replicating legacy behaviour."""
        self.stop()
        self._halt_flag = False
        self.controller.send("SH")
        self._homing_seq.clear()
        self._homing_seq.append((2, "JOG"))
        self._homing_seq.append((2, "JOG FINISHED"))
        self._homing_seq.append([1, "MOVE LIMIT"])
        self._homing_seq.append([0, "MOVE LIMIT"])
        self._homing_seq.append("RESET_AXES")
        if pack_and_go:
            self._homing_seq.append("PACK_AND_GO")

    # ──────────────────────────────────────────────────────────────────────────
    # Safety and helpers
    # ──────────────────────────────────────────────────────────────────────────
    def stop(self) -> None:
        self.controller.send("ST")

    def estop(self) -> None:
        self._halt_flag = True
        self.stop()
        self.controller.send("KP*=0")
        self.controller.send("KD*=0")
        self.controller.send("KI*=0")

    def update_status(self) -> None:
        """Poll current pulses and limit flags."""
        for ax in self.axes:
            resp = self.controller.send(f"MG_TP{ax.controller_name}")
            try:
                ax.cur_pulses = float(resp) if resp else ax.cur_pulses
            except ValueError:
                pass
            # (LR/LF/HM polling can be added here)

    def set_target_xyz(self, x: float, y: float, z: float) -> None:
        self._target_robot_xyz = [x, y, z]

    # ──────────────────────────────────────────────────────────────────────────
    # Internal math & queuing
    # ──────────────────────────────────────────────────────────────────────────
    def _current_joint_vector(self) -> AxisVec:
        return [
            self.axes[0].cur_pulses * self.axes[0].scale,
            self.axes[1].cur_pulses * self.axes[1].scale,
            self.axes[2].cur_pulses * self.axes[2].scale,
        ]

    def _check_bounds(self, pulses: AxisVec) -> None:
        for idx, p in enumerate(pulses):
            if not self.axes[idx].within_limits(p):
                raise ValueError(f"Axis {idx} target {p} outside limits")

    def _check_collision(self, pulses: AxisVec) -> None:
        # s = B - A in mm
        s_mm = (pulses[1] * self.axes[1].scale) - (pulses[0] * self.axes[0].scale)
        if s_mm >= self.params.s_max:
            raise ValueError("Collision: inner/outer risk")

    def _push_axis_vector(self, eng: AxisVec, *, interpolate: bool) -> None:
        # convert mm/deg → pulses
        pulses = [
            eng[0] / self.axes[0].scale,
            eng[1] / self.axes[1].scale,
            eng[2] / self.axes[2].scale,
        ]
        # safety checks
        self._check_bounds(pulses)
        self._check_collision(pulses)

        self._motion_seq.clear()
        if interpolate:
            start_mm = self.axes[0].cur_pulses * self.axes[0].scale
            finish_mm = eng[0]
            step = 5 * math.copysign(1, finish_mm - start_mm)
            mm_values = list(np.arange(start_mm, finish_mm, step))
            if not mm_values or mm_values[-1] != finish_mm:
                mm_values.append(finish_mm)
            for mm in mm_values:
                pA = mm / self.axes[0].scale
                pB = (mm + (eng[1] - eng[0])) / self.axes[1].scale
                # Inner (B) then Outer (A)
                self._motion_seq.append((1, float(pB)))
                self._motion_seq.append((0, float(pA)))
        else:
            # direct A, B, C ordering
            self._motion_seq.extend([
                (0, float(pulses[0])),
                (1, float(pulses[1])),
                (2, float(pulses[2])),
            ])

    def _step_motion_sequence(self) -> None:
        if not self._motion_seq or self._halt_flag:
            return
        idx, tgt = self._motion_seq[0]
        ax = self.axes[idx]
        ax.target_pulses = tgt
        self.controller.send(f"PA{ax.controller_name}={int(tgt)}")
        self.controller.send(f"BG{ax.controller_name}")
        self.update_status()
        err = ax.cur_pulses - ax.target_pulses
        if abs(err) > ax.err_thresh:
            self._valve_control(idx, err)
        else:
            self._motion_seq.pop(0)
            self._valve_control(idx, 0.0)

    def _valve_control(self, axis_idx: int, err: float) -> None:
        ax = self.axes[axis_idx]
        kp = 0 if abs(err) <= ax.err_thresh else 1000
        self.controller.send(f"KP{ax.controller_name}={kp}")
        self.controller.send(f"KD{ax.controller_name}=0")
        self.controller.send(f"KI{ax.controller_name}=0")

    def _step_homing_sequence(self) -> None:
        if not self._homing_seq or self._halt_flag:
            return
        step = self._homing_seq[0]
        # C‑axis jog to limit
        if isinstance(step, tuple) and step == (2, "JOG"):
            ax = self.axes[2]
            self.controller.send(f"KP{ax.controller_name}=50")
            self.controller.send(f"JG{ax.controller_name}={ax.homing_jg}")
            self.controller.send(f"HV{ax.controller_name}={ax.homing_hv}")
            self.controller.send(f"FI{ax.controller_name}")
            self.controller.send(f"BG{ax.controller_name}")
            self._homing_seq.pop(0)
            self._timer_homing.start(30000)
        # finish jog
        elif isinstance(step, tuple) and step == (2, "JOG FINISHED"):
            self._timer_homing.start(100)
            self._homing_seq.pop(0)
        # move to limit flags
        elif isinstance(step, list) and step[1] == "MOVE LIMIT":
            axis_idx = step[0]
            ax = self.axes[axis_idx]
            self.controller.send(f"PA{ax.controller_name}={ax.lower_limit}")
            self.controller.send(f"BG{ax.controller_name}")
            self._homing_seq.pop(0)
        # reset counters
        elif step == "RESET_AXES":
            for ax in self.axes:
                self.controller.send(f"DP{ax.controller_name}=0")
                ax.cur_pulses = 0
                ax.target_pulses = 0
            self._homing_seq.pop(0)
        # pack‑and‑go
        elif step == "PACK_AND_GO":
            self.insert(distance_mm=-self.params.s_max)
            self._homing_seq.pop(0)