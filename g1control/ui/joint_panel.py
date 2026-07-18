"""Seitenmenü für das ausgewählte Gelenk.

Erscheint, sobald im 3D-Modell (oder in der Gelenkliste) ein Gelenk
ausgewählt wurde, und bietet einen Schieberegler über den vollen
offiziellen Bewegungsbereich des Gelenks.
"""

from __future__ import annotations

import math

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from ..model.joints import BY_INDEX, JointSpec

SLIDER_STEPS = 2000


class JointPanel(QWidget):
    """Detailansicht + Schieberegler für ein einzelnes Gelenk."""

    targetChanged = Signal(int, float)   # (Motorindex, Sollwinkel rad)
    goZeroRequested = Signal(int)
    captureRequested = Signal(int)       # Ist-Position als Sollwert übernehmen

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.spec: JointSpec | None = None
        self._updating = False

        layout = QVBoxLayout(self)

        self._placeholder = QLabel(
            "Kein Gelenk ausgewählt.\n\n"
            "Im 3D-Modell auf ein Gelenk klicken\n"
            "oder links in der Liste auswählen."
        )
        self._placeholder.setAlignment(Qt.AlignCenter)
        self._placeholder.setWordWrap(True)
        layout.addWidget(self._placeholder)

        self._box = QGroupBox("Gelenk")
        box_layout = QVBoxLayout(self._box)

        form = QFormLayout()
        self.lbl_index = QLabel("–")
        self.lbl_group = QLabel("–")
        self.lbl_range = QLabel("–")
        self.lbl_torque = QLabel("–")
        self.lbl_actual = QLabel("–")
        form.addRow("Motorindex:", self.lbl_index)
        form.addRow("Gruppe:", self.lbl_group)
        form.addRow("Bereich:", self.lbl_range)
        form.addRow("Max. Moment:", self.lbl_torque)
        form.addRow("Ist-Position:", self.lbl_actual)
        box_layout.addLayout(form)

        box_layout.addSpacing(8)
        box_layout.addWidget(QLabel("Sollposition:"))

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, SLIDER_STEPS)
        self.slider.valueChanged.connect(self._on_slider)
        box_layout.addWidget(self.slider)

        minmax = QHBoxLayout()
        self.lbl_min = QLabel("")
        self.lbl_max = QLabel("")
        self.lbl_max.setAlignment(Qt.AlignRight)
        minmax.addWidget(self.lbl_min)
        minmax.addStretch(1)
        minmax.addWidget(self.lbl_max)
        box_layout.addLayout(minmax)

        spin_row = QHBoxLayout()
        self.spin_deg = QDoubleSpinBox()
        self.spin_deg.setSuffix(" °")
        self.spin_deg.setDecimals(1)
        self.spin_deg.setSingleStep(1.0)
        # Erst beim Bestätigen (Enter/Fokuswechsel) senden — sonst gehen
        # beim Tippen von "150" die Zwischenwerte 1° und 15° als echte
        # Sollwerte an den Roboter.
        self.spin_deg.setKeyboardTracking(False)
        self.spin_deg.valueChanged.connect(self._on_spin)
        self.lbl_rad = QLabel("")
        spin_row.addWidget(self.spin_deg)
        spin_row.addWidget(self.lbl_rad)
        spin_row.addStretch(1)
        box_layout.addLayout(spin_row)

        btn_row = QHBoxLayout()
        self.btn_zero = QPushButton("Auf 0°")
        self.btn_zero.clicked.connect(lambda: self.spec and self.goZeroRequested.emit(self.spec.index))
        self.btn_capture = QPushButton("Ist übernehmen")
        self.btn_capture.setToolTip("Aktuelle Ist-Position als Sollwert übernehmen")
        self.btn_capture.clicked.connect(lambda: self.spec and self.captureRequested.emit(self.spec.index))
        btn_row.addWidget(self.btn_zero)
        btn_row.addWidget(self.btn_capture)
        box_layout.addLayout(btn_row)

        self.lbl_hint = QLabel("")
        self.lbl_hint.setWordWrap(True)
        self.lbl_hint.setStyleSheet("color: #c98a00;")
        box_layout.addWidget(self.lbl_hint)

        layout.addWidget(self._box)
        layout.addStretch(1)
        self._box.hide()

    # ------------------------------------------------------------------
    def show_joint(self, index: int | None, current_target: float = 0.0) -> None:
        if index is None:
            self.spec = None
            self._box.hide()
            self._placeholder.show()
            return
        self.spec = BY_INDEX[index]
        s = self.spec
        lo_d, up_d = s.deg()
        self._box.setTitle(f"{s.name}  ({s.urdf_name})")
        self.lbl_index.setText(str(s.index))
        self.lbl_group.setText(s.group)
        self.lbl_range.setText(f"{s.lower:+.3f} … {s.upper:+.3f} rad  ({lo_d:+.1f}° … {up_d:+.1f}°)")
        self.lbl_torque.setText(f"{s.max_torque:.0f} Nm,  max. {s.max_velocity:.0f} rad/s")
        self.lbl_min.setText(f"{lo_d:+.1f}°")
        self.lbl_max.setText(f"{up_d:+.1f}°")
        self._updating = True
        self.spin_deg.setRange(lo_d, up_d)
        self._updating = False
        self.set_target_display(current_target)
        self._placeholder.hide()
        self._box.show()

    def set_target_display(self, q_rad: float) -> None:
        """Slider/Spinbox auf einen Wert setzen, ohne Signale auszulösen."""
        if self.spec is None:
            return
        self._updating = True
        frac = (self.spec.clamp(q_rad) - self.spec.lower) / self.spec.range
        self.slider.setValue(round(frac * SLIDER_STEPS))
        self.spin_deg.setValue(math.degrees(q_rad))
        self.lbl_rad.setText(f"= {q_rad:+.3f} rad")
        self._updating = False

    def set_actual(self, q_rad: float) -> None:
        self.lbl_actual.setText(f"{math.degrees(q_rad):+.1f}°  ({q_rad:+.3f} rad)")

    def set_hint(self, text: str) -> None:
        self.lbl_hint.setText(text)

    def set_slider_enabled(self, enabled: bool) -> None:
        self.slider.setEnabled(enabled)
        self.spin_deg.setEnabled(enabled)
        self.btn_zero.setEnabled(enabled)
        self.btn_capture.setEnabled(enabled)

    # ------------------------------------------------------------------
    def _on_slider(self, value: int) -> None:
        if self._updating or self.spec is None:
            return
        q = self.spec.lower + (value / SLIDER_STEPS) * self.spec.range
        self._updating = True
        self.spin_deg.setValue(math.degrees(q))
        self.lbl_rad.setText(f"= {q:+.3f} rad")
        self._updating = False
        self.targetChanged.emit(self.spec.index, q)

    def _on_spin(self, deg: float) -> None:
        if self._updating or self.spec is None:
            return
        q = self.spec.clamp(math.radians(deg))
        self._updating = True
        frac = (q - self.spec.lower) / self.spec.range
        self.slider.setValue(round(frac * SLIDER_STEPS))
        self.lbl_rad.setText(f"= {q:+.3f} rad")
        self._updating = False
        self.targetChanged.emit(self.spec.index, q)
