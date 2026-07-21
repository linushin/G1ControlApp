"""Handsteuerung — Platzhalter.

Der G1 Edu kann mit verschiedenen Händen ausgestattet werden (z. B.
Dex3-1 mit 7 DOF pro Hand oder Inspire-Hände). Das genaue Handmodell wird
später integriert; diese Schnittstelle definiert bereits jetzt die
API, gegen die die UI programmiert ist.

Für die spätere Integration (z. B. Dex3-1) muss lediglich eine weitere
Unterklasse von :class:`HandInterface` implementiert werden, die die
Befehle über die entsprechenden DDS-Topics (z. B. ``rt/dex3/left/cmd``)
verschickt.
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass

LOG = logging.getLogger(__name__)

LEFT = "left"
RIGHT = "right"


@dataclass(frozen=True)
class HandDofSpec:
    """Ein Freiheitsgrad der (Platzhalter-)Hand, Werte normiert 0..1."""

    name: str
    default: float = 0.0


class HandInterface(ABC):
    """Abstrakte Schnittstelle für beliebige Handmodelle."""

    #: Freiheitsgrade pro Hand — vom konkreten Handmodell definiert.
    dofs: list[HandDofSpec] = []

    @abstractmethod
    def set_dof(self, side: str, dof_index: int, value: float) -> None:
        """Einen Freiheitsgrad (0..1 normiert) einer Hand setzen."""

    @abstractmethod
    def open_hand(self, side: str) -> None:
        """Hand vollständig öffnen."""

    @abstractmethod
    def close_hand(self, side: str) -> None:
        """Hand vollständig schließen."""


class PlaceholderHand(HandInterface):
    """Platzhalter: nimmt Befehle entgegen und protokolliert sie nur.

    TODO: Durch die Implementierung des tatsächlichen Handmodells
    (z. B. Dex3-1) ersetzen, sobald dieses feststeht.
    """

    dofs = [
        HandDofSpec("Greifen (öffnen/schließen)"),
        HandDofSpec("Daumen"),
        HandDofSpec("Zeigefinger"),
        HandDofSpec("übrige Finger"),
    ]

    def __init__(self) -> None:
        self.values = {
            LEFT: [d.default for d in self.dofs],
            RIGHT: [d.default for d in self.dofs],
        }

    def set_dof(self, side: str, dof_index: int, value: float) -> None:
        value = min(1.0, max(0.0, value))
        self.values[side][dof_index] = value
        LOG.info("[Hand-Platzhalter] %s: %s = %.2f (kein Versand — Handmodell folgt)",
                 side, self.dofs[dof_index].name, value)

    def open_hand(self, side: str) -> None:
        for i in range(len(self.dofs)):
            self.set_dof(side, i, 0.0)

    def close_hand(self, side: str) -> None:
        for i in range(len(self.dofs)):
            self.set_dof(side, i, 1.0)
