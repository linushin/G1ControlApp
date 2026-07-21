"""Gelenk-Definitionen des Unitree G1 (29 DOF).

Indizes, Namen und Winkelgrenzen entsprechen der offiziellen
Unitree-Dokumentation (G1 Developer Guide, support.unitree.com) bzw. der
offiziellen 29-DOF-URDF (g1_29dof.urdf) von Unitree Robotics.

Alle Winkel in Radiant, Drehmomente in Nm, Geschwindigkeiten in rad/s.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


class G1JointIndex:
    """Motor-Indizes des G1 (29 DOF), identisch zur offiziellen SDK-Belegung."""

    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4   # PR-Modus (entspricht Ankle B im AB-Modus)
    LeftAnkleRoll = 5    # PR-Modus (entspricht Ankle A im AB-Modus)
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11
    WaistYaw = 12
    WaistRoll = 13       # bei 23-DOF-Variante gesperrt
    WaistPitch = 14      # bei 23-DOF-Variante gesperrt
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20  # bei 23-DOF-Variante gesperrt
    LeftWristYaw = 21    # bei 23-DOF-Variante gesperrt
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28


NUM_JOINTS = 29

# Gruppen für die Baumansicht / Farbgebung
GROUP_LEFT_LEG = "Linkes Bein"
GROUP_RIGHT_LEG = "Rechtes Bein"
GROUP_WAIST = "Hüfte / Taille"
GROUP_LEFT_ARM = "Linker Arm"
GROUP_RIGHT_ARM = "Rechter Arm"

GROUPS = [GROUP_LEFT_LEG, GROUP_RIGHT_LEG, GROUP_WAIST, GROUP_LEFT_ARM, GROUP_RIGHT_ARM]


@dataclass(frozen=True)
class JointSpec:
    index: int
    name: str            # Anzeigename
    urdf_name: str       # Name in der offiziellen URDF
    group: str
    lower: float         # Untere Winkelgrenze [rad]
    upper: float         # Obere Winkelgrenze [rad]
    max_torque: float    # [Nm]
    max_velocity: float  # [rad/s]
    kp: float            # Standard-Positionsverstärkung (aus offiziellem Beispiel)
    kd: float            # Standard-Dämpfung

    @property
    def range(self) -> float:
        return self.upper - self.lower

    def clamp(self, q: float) -> float:
        return min(self.upper, max(self.lower, q))

    def deg(self) -> tuple[float, float]:
        return math.degrees(self.lower), math.degrees(self.upper)


def _j(idx, name, urdf, group, lo, up, tau, vel, kp, kd):
    return JointSpec(idx, name, urdf, group, lo, up, tau, vel, kp, kd)


# Limits: offizielle Unitree g1_29dof.urdf; Kp/Kd: offizielles
# g1_low_level_example der unitree_sdk2_python.
JOINTS: list[JointSpec] = [
    # --- Linkes Bein ---
    _j(0,  "Hüfte Pitch links",    "left_hip_pitch_joint",    GROUP_LEFT_LEG,  -2.5307, 2.8798, 88, 32, 60, 1),
    _j(1,  "Hüfte Roll links",     "left_hip_roll_joint",     GROUP_LEFT_LEG,  -0.5236, 2.9671, 88, 32, 60, 1),
    _j(2,  "Hüfte Yaw links",      "left_hip_yaw_joint",      GROUP_LEFT_LEG,  -2.7576, 2.7576, 88, 32, 60, 1),
    _j(3,  "Knie links",           "left_knee_joint",         GROUP_LEFT_LEG,  -0.087267, 2.8798, 139, 20, 100, 2),
    _j(4,  "Knöchel Pitch links",  "left_ankle_pitch_joint",  GROUP_LEFT_LEG,  -0.87267, 0.5236, 35, 30, 40, 1),
    _j(5,  "Knöchel Roll links",   "left_ankle_roll_joint",   GROUP_LEFT_LEG,  -0.2618, 0.2618, 35, 30, 40, 1),
    # --- Rechtes Bein ---
    _j(6,  "Hüfte Pitch rechts",   "right_hip_pitch_joint",   GROUP_RIGHT_LEG, -2.5307, 2.8798, 88, 32, 60, 1),
    _j(7,  "Hüfte Roll rechts",    "right_hip_roll_joint",    GROUP_RIGHT_LEG, -2.9671, 0.5236, 88, 32, 60, 1),
    _j(8,  "Hüfte Yaw rechts",     "right_hip_yaw_joint",     GROUP_RIGHT_LEG, -2.7576, 2.7576, 88, 32, 60, 1),
    _j(9,  "Knie rechts",          "right_knee_joint",        GROUP_RIGHT_LEG, -0.087267, 2.8798, 139, 20, 100, 2),
    _j(10, "Knöchel Pitch rechts", "right_ankle_pitch_joint", GROUP_RIGHT_LEG, -0.87267, 0.5236, 35, 30, 40, 1),
    _j(11, "Knöchel Roll rechts",  "right_ankle_roll_joint",  GROUP_RIGHT_LEG, -0.2618, 0.2618, 35, 30, 40, 1),
    # --- Taille ---
    _j(12, "Taille Yaw",           "waist_yaw_joint",         GROUP_WAIST,     -2.618, 2.618, 88, 32, 60, 1),
    _j(13, "Taille Roll",          "waist_roll_joint",        GROUP_WAIST,     -0.52, 0.52, 35, 30, 40, 1),
    _j(14, "Taille Pitch",         "waist_pitch_joint",       GROUP_WAIST,     -0.52, 0.52, 35, 30, 40, 1),
    # --- Linker Arm ---
    _j(15, "Schulter Pitch links", "left_shoulder_pitch_joint", GROUP_LEFT_ARM, -3.0892, 2.6704, 25, 37, 40, 1),
    _j(16, "Schulter Roll links",  "left_shoulder_roll_joint",  GROUP_LEFT_ARM, -1.5882, 2.2515, 25, 37, 40, 1),
    _j(17, "Schulter Yaw links",   "left_shoulder_yaw_joint",   GROUP_LEFT_ARM, -2.618, 2.618, 25, 37, 40, 1),
    _j(18, "Ellbogen links",       "left_elbow_joint",          GROUP_LEFT_ARM, -1.0472, 2.0944, 25, 37, 40, 1),
    _j(19, "Handgelenk Roll links",  "left_wrist_roll_joint",   GROUP_LEFT_ARM, -1.972222054, 1.972222054, 25, 37, 40, 1),
    _j(20, "Handgelenk Pitch links", "left_wrist_pitch_joint",  GROUP_LEFT_ARM, -1.614429558, 1.614429558, 5, 22, 40, 1),
    _j(21, "Handgelenk Yaw links",   "left_wrist_yaw_joint",    GROUP_LEFT_ARM, -1.614429558, 1.614429558, 5, 22, 40, 1),
    # --- Rechter Arm ---
    _j(22, "Schulter Pitch rechts", "right_shoulder_pitch_joint", GROUP_RIGHT_ARM, -3.0892, 2.6704, 25, 37, 40, 1),
    _j(23, "Schulter Roll rechts",  "right_shoulder_roll_joint",  GROUP_RIGHT_ARM, -2.2515, 1.5882, 25, 37, 40, 1),
    _j(24, "Schulter Yaw rechts",   "right_shoulder_yaw_joint",   GROUP_RIGHT_ARM, -2.618, 2.618, 25, 37, 40, 1),
    _j(25, "Ellbogen rechts",       "right_elbow_joint",          GROUP_RIGHT_ARM, -1.0472, 2.0944, 25, 37, 40, 1),
    _j(26, "Handgelenk Roll rechts",  "right_wrist_roll_joint",   GROUP_RIGHT_ARM, -1.972222054, 1.972222054, 25, 37, 40, 1),
    _j(27, "Handgelenk Pitch rechts", "right_wrist_pitch_joint",  GROUP_RIGHT_ARM, -1.614429558, 1.614429558, 5, 22, 40, 1),
    _j(28, "Handgelenk Yaw rechts",   "right_wrist_yaw_joint",    GROUP_RIGHT_ARM, -1.614429558, 1.614429558, 5, 22, 40, 1),
]

assert len(JOINTS) == NUM_JOINTS
assert [j.index for j in JOINTS] == list(range(NUM_JOINTS))

BY_URDF_NAME = {j.urdf_name: j for j in JOINTS}
BY_INDEX = {j.index: j for j in JOINTS}


def joints_in_group(group: str) -> list[JointSpec]:
    return [j for j in JOINTS if j.group == group]
