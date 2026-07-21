"""Dialog zum Verwalten mehrerer G1-Roboterprofile.

Jedes Profil beschreibt einen Roboter mit Name, Verbindungsart (aktuell
Ethernet), Netzwerkschnittstelle und DDS-Domain. Die Profile werden
dauerhaft gespeichert.
"""

from __future__ import annotations

from pathlib import Path

from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
)

from ..robot.profiles import ProfileStore, RobotProfile


def _network_interfaces() -> list[str]:
    """Verfügbare Netzwerkschnittstellen des Systems (Linux)."""
    try:
        return sorted(p.name for p in Path("/sys/class/net").iterdir() if p.name != "lo")
    except OSError:
        return []


class ConnectionDialog(QDialog):
    def __init__(self, store: ProfileStore, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Roboter verwalten")
        self.store = store
        self._current = -1

        layout = QHBoxLayout(self)

        left = QVBoxLayout()
        self.list = QListWidget()
        self.list.currentRowChanged.connect(self._on_select)
        left.addWidget(self.list)
        btns = QHBoxLayout()
        b_add = QPushButton("Neu")
        b_del = QPushButton("Entfernen")
        b_add.clicked.connect(self._add)
        b_del.clicked.connect(self._remove)
        btns.addWidget(b_add)
        btns.addWidget(b_del)
        left.addLayout(btns)
        layout.addLayout(left, 1)

        right = QVBoxLayout()
        form = QFormLayout()
        self.ed_name = QLineEdit()
        self.ed_name.editingFinished.connect(self._apply_fields)
        self.cb_conn = QComboBox()
        self.cb_conn.addItem("Ethernet (Kabel)", "ethernet")
        self.cb_iface = QComboBox()
        self.cb_iface.setEditable(True)
        self.cb_iface.addItems(_network_interfaces())
        self.cb_iface.currentTextChanged.connect(self._apply_fields)
        self.sp_domain = QSpinBox()
        self.sp_domain.setRange(0, 232)
        self.sp_domain.valueChanged.connect(self._apply_fields)
        form.addRow("Name:", self.ed_name)
        form.addRow("Verbindung:", self.cb_conn)
        form.addRow("Schnittstelle:", self.cb_iface)
        form.addRow("DDS-Domain:", self.sp_domain)
        right.addLayout(form)
        hint = QLabel(
            "Ethernet-Direktverbindung: PC-Schnittstelle auf eine statische "
            "IP im Netz 192.168.123.x konfigurieren (z. B. 192.168.123.99/24). "
            "Der G1 ist unter 192.168.123.164 erreichbar."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: gray;")
        right.addWidget(hint)
        right.addStretch(1)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self._accept)
        bb.rejected.connect(self.reject)
        right.addWidget(bb)
        layout.addLayout(right, 2)

        self._reload_list()
        self.list.setCurrentRow(self.store.active)

    # ------------------------------------------------------------------
    def _reload_list(self) -> None:
        self.list.blockSignals(True)
        self.list.clear()
        for p in self.store.profiles:
            self.list.addItem(f"{p.name}  [{p.interface}]")
        self.list.blockSignals(False)

    def _on_select(self, row: int) -> None:
        self._apply_fields()
        self._current = row
        if 0 <= row < len(self.store.profiles):
            p = self.store.profiles[row]
            self.ed_name.setText(p.name)
            self.cb_iface.setCurrentText(p.interface)
            self.sp_domain.setValue(p.domain_id)

    def _apply_fields(self) -> None:
        row = self._current
        if 0 <= row < len(self.store.profiles):
            p = self.store.profiles[row]
            if self.ed_name.text().strip():
                p.name = self.ed_name.text().strip()
            if self.cb_iface.currentText().strip():
                p.interface = self.cb_iface.currentText().strip()
            p.domain_id = self.sp_domain.value()
            item = self.list.item(row)
            if item is not None:
                item.setText(f"{p.name}  [{p.interface}]")

    def _add(self) -> None:
        self._apply_fields()
        self.store.profiles.append(RobotProfile(name=f"G1 #{len(self.store.profiles) + 1}"))
        self._reload_list()
        self.list.setCurrentRow(len(self.store.profiles) - 1)

    def _remove(self) -> None:
        row = self.list.currentRow()
        if len(self.store.profiles) <= 1 or row < 0:
            return
        self._current = -1
        del self.store.profiles[row]
        self.store.active = min(self.store.active, len(self.store.profiles) - 1)
        self._reload_list()
        self.list.setCurrentRow(0)

    def _accept(self) -> None:
        self._apply_fields()
        row = self.list.currentRow()
        if row >= 0:
            self.store.active = row
        self.store.save()
        self.accept()
