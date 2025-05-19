from __future__ import annotations

from typing import Protocol, runtime_checkable, Optional, Tuple, Any
from abc import ABC, abstractmethod


# ---------------------------------------------------------------------------
# 1) Low‑level I/O contract
# ---------------------------------------------------------------------------
@runtime_checkable
class BaseController(Protocol):
    # ---------------------------------------------------------------------
    # Lifecycle helpers
    # ---------------------------------------------------------------------
    def connect(self, *, ip: Optional[str] = None, port: Optional[int] = None, **kwargs: Any) -> bool:
        """Open the underlying transport.
        """

    def disconnect(self) -> None:
        """Close the transport and release resources."""

    # ---------------------------------------------------------------------
    # Synchronous command helpers
    # ---------------------------------------------------------------------
    def send(self, cmd: str) -> Optional[str]:
        """Send *cmd* and return the raw response (without terminator) if any.
        """

    # ---------------------------------------------------------------------
    # Informational helpers (optional)
    # ---------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:  
        """Whether *connect()* has succeeded and the transport is open."""


# ---------------------------------------------------------------------------
# 2) High‑level robot contract
# ---------------------------------------------------------------------------
class BaseRobot(ABC):
    """Abstract base class that every concrete robot must subclass.
    """

    def __init__(self, controller: BaseController):
        if not isinstance(controller, BaseController):  # runtime check
            raise TypeError("controller must implement BaseController")
        self._controller = controller

    # ---------------------------------------------------------------------
    # Mandatory initialisation
    # ---------------------------------------------------------------------
    @abstractmethod
    def initial_config(self) -> None:
        """Push all tuning parameters (KP, KD, etc.) to the controller."""

    # ---------------------------------------------------------------------
    # Core motion primitives – UI relies on these names
    # ---------------------------------------------------------------------
    @abstractmethod
    def insert(self, *, distance_mm: Optional[float] = None) -> None:  # Jog / Insert
        """Move the robot by *distance_mm* or compute from UI target."""

    @abstractmethod
    def move_all_axes(self) -> None:
        """Plan a 3‑axis synchronous move based on UI target values."""

    @abstractmethod
    def home(self, *, pack_and_go: bool = False) -> None:
        """Homing sequence.  If *pack_and_go* is *True* finish in folded pose."""

    # ---------------------------------------------------------------------
    # Safety / control helpers
    # ---------------------------------------------------------------------
    @abstractmethod
    def stop(self) -> None:
        """Graceful stop (ST) without zeroing gains."""

    @abstractmethod
    def estop(self) -> None:
        """Emergency stop: set gains to zero and raise internal halt flag."""

    # ---------------------------------------------------------------------
    # Status & planning helpers
    # ---------------------------------------------------------------------
    @abstractmethod
    def update_status(self) -> None:
        """Refresh current pulses, limit switches, velocities, etc."""

    def set_target_xyz(self, x: float, y: float, z: float) -> None:  
        """Accept RAS‑frame target (default implementation: no‑op)."""
        return

    # ---------------------------------------------------------------------
    # Convenience
    # ---------------------------------------------------------------------
    @property
    def controller(self) -> BaseController: 
        return self._controller