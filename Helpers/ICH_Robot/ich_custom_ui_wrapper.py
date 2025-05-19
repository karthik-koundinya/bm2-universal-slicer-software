from __future__ import annotations
from typing import Optional, Any
import qt
import slicer
import numpy as np
import math
import logging

from ..Drivers.galil_controller import GalilController
from .ich_robot import ICHRobot
from .ich_data import AxisVec, AxisStatus

_LOG = logging.getLogger(__name__)

class ICH_CustomUIWrapper(qt.QObject):
    def __init__(self, ui_path: str, parent: Optional[qt.QWidget] = None) -> None:
        super().__init__(parent)
        # Load the UI file
        self._widget = self._load_ui(ui_path, parent)
        self._controller: Optional[GalilController] = None
        self._robot: Optional[ICHRobot] = None
        self._editing_target_eng = False

        self._lookup_widgets()
        self._connect_signals()

        # Timer to refresh GUI
        self._timer_gui = qt.QTimer()
        self._timer_gui.timeout.connect(self._update_gui)

    def get_widget(self) -> qt.QWidget:
        return self._widget

    def _load_ui(self, path: str, parent: Optional[qt.QWidget]) -> qt.QWidget:
        loader = qt.QUiLoader()
        file = qt.QFile(path)
        if not file.open(qt.QFile.ReadOnly):
            raise RuntimeError(f"Cannot open UI file: {path}")
        widget = loader.load(file, parent)
        file.close()
        if widget is None:
            raise RuntimeError("Failed to load UI")
        return widget

    def _w(self, name: str) -> Any:
        w = self._widget.findChild(qt.QWidget, name)
        if w is None:
            raise AttributeError(f"Widget '{name}' not found")
        return w

    def _lookup_widgets(self) -> None:
        # Connection controls
        self.wip = self._w("controller_ip")
        self.btn_connect = self._w("connect_con")
        self.btn_estop = self._w("eStopBtn")
        self.btn_enable = self._w("enable_sh")
        # Expert control
        self.cmb_axis = self._w("axisNameCombo")
        self.txt_cur_pulses = self._w("curPulsesLine")
        self.txt_cur_eng = self._w("curEngLine")
        self.txt_target_pulses = self._w("targetPulsesLine")
        self.txt_target_eng = self._w("targetEngLine")
        self.btn_set = self._w("setBtn")
        self.btn_reset = self._w("resetBtn")
        # Operation LED
        self.lbl_op_flag = self._w("OperationFlagIndicator")

    def _connect_signals(self) -> None:
        self.btn_connect.connect("clicked()", self._on_connect)
        self.btn_estop.connect("clicked()", self._on_estop)
        self.btn_enable.connect("clicked()", self._on_enable)

        # Target edits
        self.txt_target_eng.editingFinished.connect(self._on_target_eng_edited)
        self.txt_target_eng.installEventFilter(self)

        # Set & Reset
        self.btn_set.connect("clicked()", self._on_set)
        self.btn_reset.connect("clicked()", self._on_reset_axis)

    def eventFilter(self, obj: Any, event: qt.QEvent) -> bool:
        if obj is self.txt_target_eng:
            if event.type() == qt.QEvent.FocusIn:
                self._editing_target_eng = True
            elif event.type() == qt.QEvent.FocusOut:
                self._editing_target_eng = False
        return False

    # ------------------------
    # Connection / Enable / E‑Stop
    # ------------------------
    def _on_connect(self) -> None:
        if self._controller and self._controller.is_connected:
            # Disconnect flow
            self._timer_gui.stop()
            self._controller.disconnect()
            self.btn_connect.text = "Connect"
            return

        # Connect
        ip = self.wip.text.strip() or "192.168.42.100"
        self._controller = GalilController()
        if not self._controller.connect(ip=ip, port=23):
            slicer.util.errorDisplay("Unable to connect to Galil controller")
            self._controller = None
            return

        # Create robot, configure it, start GUI updates
        self._robot = ICHRobot(self._controller)
        self._robot.initial_config()
        self.btn_connect.text = "Disconnect"
        self.lbl_op_flag.setStyleSheet("background-color:red")
        self._timer_gui.start(50)

    def _on_enable(self) -> None:
        if not self._robot:
            slicer.util.errorDisplay("Robot not connected")
            return
        # Servo on
        self._robot.controller.send("SH")
        self.lbl_op_flag.setStyleSheet("background-color:green")

    def _on_estop(self) -> None:
        if self._robot:
            self._robot.estop()
            self.lbl_op_flag.setStyleSheet("background-color:red")

    # ------------------------
    # Expert control handlers
    # ------------------------
    def _current_axis_idx(self) -> int:
        return self.cmb_axis.currentIndex

    def _axis(self, idx: int) -> AxisStatus:
        if not self._robot:
            raise RuntimeError("Robot not connected")
        return self._robot.axes[idx]

    def _on_target_eng_edited(self) -> None:
        if not self._robot:
            return
        idx = self._current_axis_idx()
        ax = self._axis(idx)
        try:
            eng = float(self.txt_target_eng.text)
            ax.target_pulses = round(eng / ax.scale)
        except ValueError:
            pass

    def _on_set(self) -> None:
        if not self._robot:
            slicer.util.errorDisplay("Robot not connected")
            return
        idx = self._current_axis_idx()
        ax = self._axis(idx)
        # Clear E‑Stop, turn servo on, then queue move
        self._robot._halt_flag = False
        self._robot.controller.send("SH")
        self._robot.move_single_axis(idx, ax.target_pulses)

    def _on_reset_axis(self) -> None:
        if not self._robot:
            return
        idx = self._current_axis_idx()
        ax = self._axis(idx)
        # Zero encoder counter
        self._robot.controller.send(f"DP{ax.controller_name}=0")
        ax.cur_pulses = ax.target_pulses = 0

    # ------------------------
    # GUI update loop (50 ms)
    # ------------------------
    def _update_gui(self) -> None:
        if not self._robot:
            return

        self._robot.update_status()

        idx = self._current_axis_idx()
        ax = self._axis(idx)

        # Current position
        self.txt_cur_pulses.text = f"{ax.cur_pulses:.0f}"
        self.txt_cur_eng.text = f"{ax.cur_pulses * ax.scale:.2f}"

        # Target position
        self.txt_target_pulses.text = f"{ax.target_pulses:.0f}"
        if not self._editing_target_eng:
            self.txt_target_eng.text = f"{ax.target_pulses * ax.scale:.2f}"

        # Validity coloring
        col = "green" if ax.within_limits(ax.target_pulses) else "red"
        self.txt_target_pulses.setStyleSheet(f"background-color: {col}")
        self.txt_target_eng.setStyleSheet(f"background-color: {col}")