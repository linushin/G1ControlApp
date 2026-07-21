"""Abstrakte Roboterschnittstelle.

Über diese Schnittstelle lassen sich verschiedene G1-Roboter (und später
weitere Verbindungsarten) einheitlich ansprechen. Aktuell implementiert:

* :class:`g1control.robot.g1_ethernet.G1EthernetRobot` — echte Verbindung
  über Ethernet mit der offiziellen unitree_sdk2_python (DDS Low-Level).
* :class:`g1control.robot.mock.MockRobot` — Simulation ohne Hardware.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np

from ..model.joints import NUM_JOINTS


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


class RobotInterface(ABC):
    """Gemeinsame Schnittstelle aller Roboter-Backends."""

    def __init__(self, name: str) -> None:
        self.name = name

    # --- Verbindung -----------------------------------------------------
    @abstractmethod
    def connect(self) -> None:
        """Verbindung aufbauen (blockierend, wirft Exception bei Fehler)."""

    @abstractmethod
    def disconnect(self) -> None:
        """Verbindung trennen; aktive Steuerung wird vorher beendet."""

    # --- Zustand --------------------------------------------------------
    @abstractmethod
    def state(self) -> RobotState:
        """Aktuellen Zustand (thread-sicher, Kopie) zurückgeben."""

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

    @abstractmethod
    def set_target(self, joint_index: int, q: float) -> None:
        """Sollposition eines Gelenks setzen (wird intern limitiert und
        mit begrenzter Geschwindigkeit angefahren)."""

    @abstractmethod
    def emergency_damp(self) -> None:
        """NOT-AUS: sofort alle Gelenke in den Dämpfungsmodus."""
