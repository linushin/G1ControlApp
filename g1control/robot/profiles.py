"""Roboterprofile — Schnittstelle zum Verwalten mehrerer G1.

Profile (Name + Netzwerkschnittstelle + DDS-Domain) werden als JSON unter
``~/.config/g1control/robots.json`` gespeichert. In der App wird jeweils
ein Profil als aktiver Roboter ausgewählt und verbunden.

Hinweis: Die DDS-Schicht der unitree_sdk2_python kann pro Prozess nur für
eine Netzwerkschnittstelle initialisiert werden. Mehrere G1 werden daher
als Profile verwaltet und nacheinander verbunden; gleichzeitige
Verbindungen zu mehreren Robotern sind ein späterer Ausbauschritt.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

CONFIG_DIR = Path.home() / ".config" / "g1control"
CONFIG_FILE = CONFIG_DIR / "robots.json"


@dataclass
class RobotProfile:
    name: str = "Mein G1"
    connection: str = "ethernet"     # aktuell einzige Verbindungsart
    interface: str = "eth0"          # Netzwerkschnittstelle, z. B. enp2s0
    domain_id: int = 0               # DDS-Domain (Standard 0)


@dataclass
class ProfileStore:
    profiles: list[RobotProfile] = field(default_factory=list)
    active: int = 0

    @classmethod
    def load(cls) -> "ProfileStore":
        try:
            data = json.loads(CONFIG_FILE.read_text())
            profiles = [RobotProfile(**p) for p in data.get("profiles", [])]
            store = cls(profiles=profiles, active=int(data.get("active", 0)))
        except (OSError, ValueError, TypeError):
            store = cls()
        if not store.profiles:
            store.profiles = [RobotProfile()]
        store.active = min(max(store.active, 0), len(store.profiles) - 1)
        return store

    def save(self) -> None:
        CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        CONFIG_FILE.write_text(json.dumps(
            {"profiles": [asdict(p) for p in self.profiles], "active": self.active},
            indent=2, ensure_ascii=False))

    def active_profile(self) -> RobotProfile:
        return self.profiles[self.active]
