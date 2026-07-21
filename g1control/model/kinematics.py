"""Kinematisches Modell des G1 für die 3D-Ansicht.

Der Gelenkbaum (Ursprünge, Orientierungen, Drehachsen) stammt aus der
offiziellen Unitree-URDF ``g1_29dof.urdf``. Die Vorwärtskinematik liefert
die Weltpositionen aller Gelenke für eine gegebene Gelenkstellung und wird
ausschließlich für die Visualisierung verwendet.

Koordinatensystem wie in der URDF: x nach vorn, y nach links, z nach oben.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class KinNode:
    """Ein Knoten des Gelenkbaums (revolutes Gelenk oder fixer Punkt)."""

    name: str
    parent: str | None
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float] | None = None  # None => fix
    joint_index: int | None = None                  # Motorindex, falls steuerbar
    children: list[str] = field(default_factory=list)


def _rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _axis_angle(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    ax = np.asarray(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = ax
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) * c + s * k + (1 - c) * np.outer(ax, ax)


# (name, parent, xyz, rpy, axis, motor_index) — aus der offiziellen g1_29dof.urdf.
_TREE_DATA = [
    ("pelvis", None, (0, 0, 0), (0, 0, 0), None, None),
    # Linkes Bein
    ("left_hip_pitch_joint", "pelvis", (0, 0.064452, -0.1027), (0, 0, 0), (0, 1, 0), 0),
    ("left_hip_roll_joint", "left_hip_pitch_joint", (0, 0.052, -0.030465), (0, -0.1749, 0), (1, 0, 0), 1),
    ("left_hip_yaw_joint", "left_hip_roll_joint", (0.025001, 0, -0.12412), (0, 0, 0), (0, 0, 1), 2),
    ("left_knee_joint", "left_hip_yaw_joint", (-0.078273, 0.0021489, -0.17734), (0, 0.1749, 0), (0, 1, 0), 3),
    ("left_ankle_pitch_joint", "left_knee_joint", (0, -9.4445e-05, -0.30001), (0, 0, 0), (0, 1, 0), 4),
    ("left_ankle_roll_joint", "left_ankle_pitch_joint", (0, 0, -0.017558), (0, 0, 0), (1, 0, 0), 5),
    ("left_foot_tip", "left_ankle_roll_joint", (0.14, 0, -0.03), (0, 0, 0), None, None),
    ("left_heel", "left_ankle_roll_joint", (-0.05, 0, -0.03), (0, 0, 0), None, None),
    # Rechtes Bein
    ("right_hip_pitch_joint", "pelvis", (0, -0.064452, -0.1027), (0, 0, 0), (0, 1, 0), 6),
    ("right_hip_roll_joint", "right_hip_pitch_joint", (0, -0.052, -0.030465), (0, -0.1749, 0), (1, 0, 0), 7),
    ("right_hip_yaw_joint", "right_hip_roll_joint", (0.025001, 0, -0.12412), (0, 0, 0), (0, 0, 1), 8),
    ("right_knee_joint", "right_hip_yaw_joint", (-0.078273, -0.0021489, -0.17734), (0, 0.1749, 0), (0, 1, 0), 9),
    ("right_ankle_pitch_joint", "right_knee_joint", (0, 9.4445e-05, -0.30001), (0, 0, 0), (0, 1, 0), 10),
    ("right_ankle_roll_joint", "right_ankle_pitch_joint", (0, 0, -0.017558), (0, 0, 0), (1, 0, 0), 11),
    ("right_foot_tip", "right_ankle_roll_joint", (0.14, 0, -0.03), (0, 0, 0), None, None),
    ("right_heel", "right_ankle_roll_joint", (-0.05, 0, -0.03), (0, 0, 0), None, None),
    # Taille / Torso
    ("waist_yaw_joint", "pelvis", (0, 0, 0), (0, 0, 0), (0, 0, 1), 12),
    ("waist_roll_joint", "waist_yaw_joint", (-0.0039635, 0, 0.035), (0, 0, 0), (1, 0, 0), 13),
    ("waist_pitch_joint", "waist_roll_joint", (0, 0, 0.019), (0, 0, 0), (0, 1, 0), 14),
    ("torso_top", "waist_pitch_joint", (0.0039635, 0, 0.35), (0, 0, 0), None, None),
    ("head", "waist_pitch_joint", (0.0039635, 0, 0.44), (0, 0, 0), None, None),
    # Linker Arm
    ("left_shoulder_pitch_joint", "waist_pitch_joint", (0.0039563, 0.10022, 0.23778), (0.27931, 5.4949e-05, -0.00019159), (0, 1, 0), 15),
    ("left_shoulder_roll_joint", "left_shoulder_pitch_joint", (0, 0.038, -0.013831), (-0.27925, 0, 0), (1, 0, 0), 16),
    ("left_shoulder_yaw_joint", "left_shoulder_roll_joint", (0, 0.00624, -0.1032), (0, 0, 0), (0, 0, 1), 17),
    ("left_elbow_joint", "left_shoulder_yaw_joint", (0.015783, 0, -0.080518), (0, 0, 0), (0, 1, 0), 18),
    ("left_wrist_roll_joint", "left_elbow_joint", (0.1, 0.00188791, -0.01), (0, 0, 0), (1, 0, 0), 19),
    ("left_wrist_pitch_joint", "left_wrist_roll_joint", (0.038, 0, 0), (0, 0, 0), (0, 1, 0), 20),
    ("left_wrist_yaw_joint", "left_wrist_pitch_joint", (0.046, 0, 0), (0, 0, 0), (0, 0, 1), 21),
    ("left_hand", "left_wrist_yaw_joint", (0.12, 0.003, 0), (0, 0, 0), None, None),
    # Rechter Arm
    ("right_shoulder_pitch_joint", "waist_pitch_joint", (0.0039563, -0.10021, 0.23778), (-0.27931, 5.4949e-05, 0.00019159), (0, 1, 0), 22),
    ("right_shoulder_roll_joint", "right_shoulder_pitch_joint", (0, -0.038, -0.013831), (0.27925, 0, 0), (1, 0, 0), 23),
    ("right_shoulder_yaw_joint", "right_shoulder_roll_joint", (0, -0.00624, -0.1032), (0, 0, 0), (0, 0, 1), 24),
    ("right_elbow_joint", "right_shoulder_yaw_joint", (0.015783, 0, -0.080518), (0, 0, 0), (0, 1, 0), 25),
    ("right_wrist_roll_joint", "right_elbow_joint", (0.1, -0.00188791, -0.01), (0, 0, 0), (1, 0, 0), 26),
    ("right_wrist_pitch_joint", "right_wrist_roll_joint", (0.038, 0, 0), (0, 0, 0), (0, 1, 0), 27),
    ("right_wrist_yaw_joint", "right_wrist_pitch_joint", (0.046, 0, 0), (0, 0, 0), (0, 0, 1), 28),
    ("right_hand", "right_wrist_yaw_joint", (0.12, -0.003, 0), (0, 0, 0), None, None),
]


class G1Kinematics:
    """Vorwärtskinematik des G1-Gelenkbaums."""

    def __init__(self) -> None:
        self.nodes: dict[str, KinNode] = {}
        self.order: list[str] = []
        for name, parent, xyz, rpy, axis, idx in _TREE_DATA:
            self.nodes[name] = KinNode(name, parent, xyz, rpy, axis, idx)
            self.order.append(name)
        for node in self.nodes.values():
            if node.parent is not None:
                self.nodes[node.parent].children.append(node.name)
        self.joint_nodes = {n.joint_index: n.name for n in self.nodes.values() if n.joint_index is not None}

    def forward(self, q: np.ndarray) -> dict[str, np.ndarray]:
        """Berechnet die 4x4-Welttransformationen aller Knoten für Gelenkwinkel q[29]."""
        transforms: dict[str, np.ndarray] = {}
        for name in self.order:
            node = self.nodes[name]
            local = np.eye(4)
            local[:3, :3] = _rpy_matrix(*node.rpy)
            local[:3, 3] = node.xyz
            if node.axis is not None and node.joint_index is not None:
                rot = np.eye(4)
                rot[:3, :3] = _axis_angle(node.axis, float(q[node.joint_index]))
                local = local @ rot
            if node.parent is None:
                transforms[name] = local
            else:
                transforms[name] = transforms[node.parent] @ local
        return transforms

    def positions(self, q: np.ndarray) -> dict[str, np.ndarray]:
        return {name: t[:3, 3] for name, t in self.forward(q).items()}

    def axis_world(self, transforms: dict[str, np.ndarray], joint_index: int) -> np.ndarray:
        """Weltrichtung der Drehachse eines Gelenks (für die Achsanzeige)."""
        name = self.joint_nodes[joint_index]
        node = self.nodes[name]
        return transforms[name][:3, :3] @ np.asarray(node.axis, dtype=float)

    def segments(self) -> list[tuple[str, str]]:
        """Alle Verbindungslinien (parent -> child) für das Strichmodell."""
        segs = []
        for name in self.order:
            node = self.nodes[name]
            if node.parent is not None:
                segs.append((node.parent, name))
        return segs
