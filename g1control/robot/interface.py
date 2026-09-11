"""Abstrakte Roboterschnittstelle.

Über diese Schnittstelle lassen sich verschiedene G1-Roboter (und später
weitere Verbindungsarten) einheitlich ansprechen. Aktuell implementiert:

* :class:`g1control.robot.g1_ethernet.G1EthernetRobot` — echte Verbindung
  über Ethernet mit der offiziellen unitree_sdk2_python (DDS Low-Level).
* :class:`g1control.robot.mock.MockRobot` — Simulation ohne Hardware.

Sicherheitsrelevante Gemeinsamkeiten (Grenzwert-Clamping in
:meth:`RobotInterface.set_target`, Slew-Rate-Schritt in
:meth:`RobotInterface._slew_step`) sind hier zentral implementiert, damit
Simulation und echter Roboter garantiert dieselbe Logik verwenden.
"""

from __future__ import annotations

import dataclasses
import math
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np

from ..model.joints import LOWER, NUM_JOINTS, UPPER


@dataclass
class RobotState:
    """Zuletzt bekannter Zustand des Roboters."""

    connected: bool = False
    control_active: bool = False
    q: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS))
    dq: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS))
    tau: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS))
    temperature: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS))
    targets: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS))
    mode_machine: int = 0
    error: str = ""

    def snapshot(self) -> "RobotState":
        """Kopie mit eigenen Arrays — für die thread-sichere Übergabe an die UI."""
        return dataclasses.replace(
            self,
            q=self.q.copy(),
            dq=self.dq.copy(),
            tau=self.tau.copy(),
            temperature=self.temperature.copy(),
            targets=self.targets.copy(),
        )


class RobotInterface(ABC):
    """Gemeinsame Schnittstelle aller Roboter-Backends."""

    #: Maximale Verfahrgeschwindigkeit der Sollwerte [rad/s] (Slew-Rate).
    MAX_SPEED = 0.6

    #: True => die UI holt vor der ersten Aktivierung eine Sicherheits-
    #: bestätigung ein (echter Roboter: Balance-Dienst wird beendet).
    NEEDS_ENABLE_CONFIRMATION = False

    def __init__(self, name: str) -> None:
        self.name = name
        self._lock = threading.Lock()
        self._state = RobotState()

    # --- Verbindung -----------------------------------------------------
    @abstractmethod
    def connect(self) -> None:
        """Verbindung aufbauen (blockierend, wirft Exception bei Fehler).

        Verbinden ist passiv: Es werden keine Befehle an den Roboter
        gesendet, nur der Ist-Zustand wird gelesen.
        """

    @abstractmethod
    def disconnect(self) -> None:
        """Verbindung trennen; aktive Steuerung wird vorher beendet."""

    # --- Zustand --------------------------------------------------------
    def state(self) -> RobotState:
        """Aktuellen Zustand (thread-sicher, Kopie) zurückgeben."""
        with self._lock:
            return self._state.snapshot()

    # --- Steuerung ------------------------------------------------------
    @abstractmethod
    def enable_control(self) -> None:
        """Low-Level-Steuerung aktivieren.

        Die aktuelle Ist-Position aller Gelenke wird als Sollwert
        übernommen (Pose wird gehalten), danach können einzelne Gelenke
        über :meth:`set_target` bewegt werden.
        """

    @abstractmethod
    def disable_control(self) -> None:
        """Steuerung beenden und in den Dämpfungsmodus wechseln."""

    def set_target(self, joint_index: int, q: float) -> None:
        """Sollposition eines Gelenks setzen.

        Wird auf die offiziellen Gelenkgrenzen begrenzt und vom Backend
        mit begrenzter Geschwindigkeit angefahren. NaN/Inf werden
        abgewiesen — ``np.clip`` würde NaN ungehindert durchreichen und
        der Wert würde bei vollem kp an den Roboter gestreamt.
        """
        q = float(q)
        if not math.isfinite(q):
            raise ValueError(
                f"Ungültiger Sollwert für Gelenk {joint_index}: {q!r}")
        q = float(np.clip(q, LOWER[joint_index], UPPER[joint_index]))
        with self._lock:
            self._state.targets[joint_index] = q

    @abstractmethod
    def emergency_damp(self) -> bool:
        """NOT-AUS: sofort alle Gelenke in den Dämpfungsmodus.

        Rückgabe ``True``, wenn der Dämpfungsbefehl den Roboter auch
        tatsächlich erreicht (Sendeschleife läuft); ``False``, wenn keine
        Befehle gesendet werden (z. B. Steuerung nie aktiviert) — die UI
        muss das dem Bediener ehrlich anzeigen.
        """

    # --- Hilfen für Backends -------------------------------------------
    @staticmethod
    def _slew_step(current: np.ndarray, targets: np.ndarray,
                   max_speed: float, dt: float) -> np.ndarray:
        """Ein Slew-Rate-begrenzter Schritt von ``current`` Richtung der
        (auf die Gelenkgrenzen geklemmten) ``targets``.

        Bewusst KEIN Clip der Position selbst: Steht ein Gelenk physisch
        außerhalb der Tabellengrenzen (Kalibrierungsversatz), würde ein
        Positions-Clip den Befehl in einem einzigen Zyklus auf die Grenze
        teleportieren — ein unratenbegrenzter Ruck bei vollem kp. So wird
        die Pose stattdessen ratenbegrenzt in die Grenzen zurückgeführt.
        """
        step = np.clip(np.clip(targets, LOWER, UPPER) - current,
                       -max_speed * dt, max_speed * dt)
        return current + step
