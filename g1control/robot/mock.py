"""Simulations-Backend ohne Hardware.

Verhält sich nach außen wie ein echter G1: Gelenke fahren mit begrenzter
Geschwindigkeit auf ihre Sollwerte. Damit lässt sich die komplette App
(3D-Modell, Auswahl, Slider) ohne Roboter testen.

Clamping, Slew-Rate (inkl. MAX_SPEED) und Zustands-Snapshots kommen aus
der gemeinsamen Basisklasse :class:`~g1control.robot.interface.RobotInterface`,
damit die Simulation dieselbe Logik durchläuft wie der echte Roboter.
"""

from __future__ import annotations

import threading
import time

from .interface import RobotInterface


class MockRobot(RobotInterface):
    RATE_HZ = 100.0

    def __init__(self, name: str = "Simulation") -> None:
        super().__init__(name)
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

    def enable_control(self) -> None:
        with self._lock:
            if not self._state.connected:
                raise RuntimeError("Nicht verbunden.")
            self._state.targets = self._state.q.copy()
            self._state.control_active = True

    def disable_control(self) -> None:
        with self._lock:
            self._state.control_active = False

    def emergency_damp(self) -> None:
        with self._lock:
            self._state.control_active = False
            self._state.targets = self._state.q.copy()

    def _loop(self) -> None:
        dt = 1.0 / self.RATE_HZ
        while self._running:
            with self._lock:
                if self._state.control_active:
                    prev = self._state.q
                    self._state.q = self._slew_step(prev, self._state.targets,
                                                    self.MAX_SPEED, dt)
                    self._state.dq = (self._state.q - prev) / dt
                else:
                    self._state.dq[:] = 0.0
            time.sleep(dt)
