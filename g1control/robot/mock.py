"""Simulations-Backend ohne Hardware.

Verhält sich nach außen wie ein echter G1: Gelenke fahren mit begrenzter
Geschwindigkeit auf ihre Sollwerte. Damit lässt sich die komplette App
(3D-Modell, Auswahl, Slider) ohne Roboter testen.
"""

from __future__ import annotations

import threading
import time

import numpy as np

from ..model.joints import JOINTS, NUM_JOINTS
from .interface import RobotInterface, RobotState

_LOWER = np.array([j.lower for j in JOINTS])
_UPPER = np.array([j.upper for j in JOINTS])


class MockRobot(RobotInterface):
    RATE_HZ = 100.0
    MAX_SPEED = 1.2  # rad/s in der Simulation

    def __init__(self, name: str = "Simulation") -> None:
        super().__init__(name)
        self._lock = threading.Lock()
        self._state = RobotState()
        self._thread: threading.Thread | None = None
        self._running = False

    def connect(self) -> None:
        with self._lock:
            if self._state.connected:
                return
            self._state.connected = True
            self._state.error = ""
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def disconnect(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        with self._lock:
            self._state.connected = False
            self._state.control_active = False

    def state(self) -> RobotState:
        with self._lock:
            s = self._state
            return RobotState(
                connected=s.connected,
                control_active=s.control_active,
                q=s.q.copy(),
                dq=s.dq.copy(),
                tau=s.tau.copy(),
                temperature=s.temperature.copy(),
                targets=s.targets.copy(),
                mode_machine=s.mode_machine,
                error=s.error,
            )

    def enable_control(self) -> None:
        with self._lock:
            self._state.targets = self._state.q.copy()
            self._state.control_active = True

    def disable_control(self) -> None:
        with self._lock:
            self._state.control_active = False

    def set_target(self, joint_index: int, q: float) -> None:
        q = float(np.clip(q, _LOWER[joint_index], _UPPER[joint_index]))
        with self._lock:
            self._state.targets[joint_index] = q

    def emergency_damp(self) -> None:
        with self._lock:
            self._state.control_active = False
            self._state.targets = self._state.q.copy()

    def _loop(self) -> None:
        dt = 1.0 / self.RATE_HZ
        while self._running:
            with self._lock:
                if self._state.control_active:
                    delta = self._state.targets - self._state.q
                    step = np.clip(delta, -self.MAX_SPEED * dt, self.MAX_SPEED * dt)
                    self._state.q = np.clip(self._state.q + step, _LOWER, _UPPER)
                    self._state.dq = step / dt
                else:
                    self._state.dq[:] = 0.0
            time.sleep(dt)
