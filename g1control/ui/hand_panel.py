"""Handsteuerung (Platzhalter-UI).

Bedient die :class:`~g1control.robot.hands.HandInterface`-Schnittstelle.
Solange nur der Platzhalter aktiv ist, werden Befehle protokolliert, aber
nicht an den Roboter gesendet. Nach Integration des tatsächlichen
Handmodells funktioniert dieses Panel unverändert weiter.
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from ..robot.hands import LEFT, RIGHT, HandInterface


class _SingleHand(QGroupBox):
    def __init__(self, title: str, side: str, hand: HandInterface, parent=None) -> None:
        super().__init__(title, parent)
        self.side = side
        self.hand = hand
        layout = QVBoxLayout(self)
        self.sliders: list[QSlider] = []
        for i, dof in enumerate(hand.dofs):
            layout.addWidget(QLabel(dof.name))
            s = QSlider(Qt.Horizontal)
            s.setRange(0, 100)
            s.setValue(round(dof.default * 100))
            s.valueChanged.connect(lambda v, idx=i: self.hand.set_dof(self.side, idx, v / 100.0))
            layout.addWidget(s)
            self.sliders.append(s)
        btns = QHBoxLayout()
        b_open = QPushButton("Öffnen")
        b_close = QPushButton("Schließen")
        b_open.clicked.connect(self._open)
        b_close.clicked.connect(self._close)
        btns.addWidget(b_open)
        btns.addWidget(b_close)
        layout.addLayout(btns)

    def _open(self) -> None:
        for s in self.sliders:
            s.setValue(0)
        self.hand.open_hand(self.side)

    def _close(self) -> None:
        for s in self.sliders:
            s.setValue(100)
        self.hand.close_hand(self.side)


class HandPanel(QWidget):
    def __init__(self, hand: HandInterface, parent=None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        note = QLabel(
            "⚠ Platzhalter: Das konkrete Handmodell (z. B. Dex3-1) ist noch "
            "nicht integriert. Befehle werden nur protokolliert."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #c98a00;")
        layout.addWidget(note)
        layout.addWidget(_SingleHand("Linke Hand", LEFT, hand))
        layout.addWidget(_SingleHand("Rechte Hand", RIGHT, hand))
        layout.addStretch(1)
