"""Hauptfenster der G1-Steuerungs-App.

Aufbau:
* Toolbar oben: Roboterprofil, Verbinden/Trennen, Steuerung aktivieren,
  NOT-AUS.
* Links: Gelenkliste nach Gruppen.
* Mitte: rotierbares 3D-Modell mit anklickbaren Gelenken.
* Rechts: Seitenmenü mit Schieberegler für das ausgewählte Gelenk sowie
  Tab für die (Platzhalter-)Handsteuerung.
"""

from __future__ import annotations

import logging

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QColor
from PySide6.QtWidgets import (
    QComboBox,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QStatusBar,
    QTabWidget,
    QToolBar,
    QTreeWidget,
    QTreeWidgetItem,
    QWidget,
)

from ..model.joints import BY_INDEX, GROUPS, NUM_JOINTS, joints_in_group
from ..robot.g1_ethernet import G1EthernetRobot, sdk_available
from ..robot.hands import PlaceholderHand
from ..robot.interface import RobotInterface
from ..robot.mock import MockRobot
from ..robot.profiles import ProfileStore
from .connection_dialog import ConnectionDialog
from .hand_panel import HandPanel
from .joint_panel import JointPanel
from .robot_view import GROUP_COLORS, RobotView

LOG = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Unitree G1 Control")
        self.resize(1280, 800)

        self.store = ProfileStore.load()
        self.robot: RobotInterface | None = None
        self.hand = PlaceholderHand()
        self.preview_q = np.zeros(NUM_JOINTS)   # Pose im Offline-Modus
        self.selected: int | None = None

        self._build_toolbar()
        self._build_central()
        self._build_statusbar()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(33)
        self._refresh_profile_combo()
        self._update_control_ui()

    # ------------------------------------------------------------- Aufbau
    def _build_toolbar(self) -> None:
        tb = QToolBar("Verbindung")
        tb.setMovable(False)
        self.addToolBar(tb)

        tb.addWidget(QLabel(" Roboter: "))
        self.cb_profile = QComboBox()
        self.cb_profile.setMinimumWidth(180)
        self.cb_profile.currentIndexChanged.connect(self._on_profile_change)
        tb.addWidget(self.cb_profile)

        act_manage = QAction("Verwalten…", self)
        act_manage.triggered.connect(self._manage_profiles)
        tb.addAction(act_manage)

        tb.addSeparator()
        self.btn_connect = QPushButton("Verbinden")
        self.btn_connect.clicked.connect(self._toggle_connect)
        tb.addWidget(self.btn_connect)

        self.btn_sim = QPushButton("Simulation")
        self.btn_sim.setCheckable(True)
        self.btn_sim.setToolTip("Ohne Hardware testen (Simulations-Backend)")
        tb.addWidget(self.btn_sim)

        tb.addSeparator()
        self.btn_control = QPushButton("Steuerung aktivieren")
        self.btn_control.setCheckable(True)
        self.btn_control.setToolTip(
            "Aktiviert die Low-Level-Positionsregelung: Ein laufender "
            "High-Level-Dienst wird freigegeben, die aktuelle Pose wird "
            "gehalten, danach bewegen die Schieberegler den Roboter."
        )
        self.btn_control.clicked.connect(self._toggle_control)
        tb.addWidget(self.btn_control)

        self.btn_estop = QPushButton("NOT-AUS (Dämpfung)")
        self.btn_estop.setStyleSheet(
            "QPushButton {background-color:#b02020; color:white; font-weight:bold; padding:4px 14px;}"
        )
        self.btn_estop.clicked.connect(self._emergency)
        tb.addWidget(self.btn_estop)

    def _build_central(self) -> None:
        splitter = QSplitter(Qt.Horizontal)

        self.tree = QTreeWidget()
        self.tree.setHeaderLabel("Gelenke (29 DOF)")
        for group in GROUPS:
            top = QTreeWidgetItem([group])
            color = GROUP_COLORS[group]
            top.setForeground(0, QColor.fromRgbF(*color))
            for spec in joints_in_group(group):
                child = QTreeWidgetItem([f"{spec.index:2d}  {spec.name}"])
                child.setData(0, Qt.UserRole, spec.index)
                top.addChild(child)
            self.tree.addTopLevelItem(top)
        self.tree.expandAll()
        self.tree.currentItemChanged.connect(self._on_tree_select)
        splitter.addWidget(self.tree)

        self.view = RobotView()
        self.view.jointClicked.connect(self._select_joint)
        self.view.selectionCleared.connect(lambda: self._select_joint(None))
        splitter.addWidget(self.view)

        self.tabs = QTabWidget()
        self.joint_panel = JointPanel()
        self.joint_panel.targetChanged.connect(self._on_target_changed)
        self.joint_panel.goZeroRequested.connect(lambda i: self._send_target(i, BY_INDEX[i].clamp(0.0), sync_ui=True))
        self.joint_panel.captureRequested.connect(self._capture_actual)
        self.tabs.addTab(self.joint_panel, "Gelenk")
        self.hand_panel = HandPanel(self.hand)
        self.tabs.addTab(self.hand_panel, "Hände (Platzhalter)")
        splitter.addWidget(self.tabs)

        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 0)
        splitter.setSizes([240, 700, 330])
        self.setCentralWidget(splitter)

    def _build_statusbar(self) -> None:
        sb = QStatusBar()
        self.lbl_status = QLabel("Nicht verbunden")
        sb.addWidget(self.lbl_status)
        self.lbl_sdk = QLabel("SDK: verfügbar" if sdk_available() else "SDK: nicht installiert (nur Simulation möglich)")
        sb.addPermanentWidget(self.lbl_sdk)
        self.setStatusBar(sb)

    # ----------------------------------------------------------- Profile
    def _refresh_profile_combo(self) -> None:
        self.cb_profile.blockSignals(True)
        self.cb_profile.clear()
        for p in self.store.profiles:
            self.cb_profile.addItem(f"{p.name} [{p.interface}]")
        self.cb_profile.setCurrentIndex(self.store.active)
        self.cb_profile.blockSignals(False)

    def _on_profile_change(self, idx: int) -> None:
        if 0 <= idx < len(self.store.profiles):
            self.store.active = idx
            self.store.save()

    def _manage_profiles(self) -> None:
        dlg = ConnectionDialog(self.store, self)
        if dlg.exec():
            self._refresh_profile_combo()

    # ------------------------------------------------------- Verbindung
    def _toggle_connect(self) -> None:
        if self.robot is not None:
            self._disconnect()
            return
        profile = self.store.active_profile()
        try:
            if self.btn_sim.isChecked():
                robot: RobotInterface = MockRobot(f"Simulation ({profile.name})")
            else:
                if not sdk_available():
                    QMessageBox.warning(
                        self, "SDK fehlt",
                        "unitree_sdk2_python ist nicht installiert.\n\n"
                        "Bitte ./install.sh ausführen oder den Simulationsmodus nutzen.")
                    return
                robot = G1EthernetRobot(profile.name, profile.interface, profile.domain_id)
            robot.connect()
        except Exception as exc:
            QMessageBox.critical(self, "Verbindung fehlgeschlagen", str(exc))
            return
        self.robot = robot
        self.btn_connect.setText("Trennen")
        self.lbl_status.setText(f"Verbunden: {robot.name}")
        LOG.info("Verbunden mit %s", robot.name)
        self._update_control_ui()

    def _disconnect(self) -> None:
        if self.robot is None:
            return
        try:
            self.robot.disconnect()
        except Exception as exc:
            LOG.warning("Fehler beim Trennen: %s", exc)
        self.robot = None
        self.btn_control.setChecked(False)
        self.btn_connect.setText("Verbinden")
        self.lbl_status.setText("Nicht verbunden")
        self._update_control_ui()

    def _toggle_control(self) -> None:
        if self.robot is None:
            self.btn_control.setChecked(False)
            return
        try:
            if self.btn_control.isChecked():
                self.robot.enable_control()
                LOG.info("Steuerung aktiviert — Pose wird gehalten.")
            else:
                self.robot.disable_control()
                LOG.info("Steuerung deaktiviert — Dämpfungsmodus.")
        except Exception as exc:
            QMessageBox.critical(self, "Fehler", str(exc))
            self.btn_control.setChecked(False)
        self._update_control_ui()

    def _emergency(self) -> None:
        if self.robot is not None:
            self.robot.emergency_damp()
            # Nicht pauschal Erfolg melden: Wenn das Senden fehlschlägt,
            # erreicht der Dämpfungsbefehl den Roboter nicht.
            err = self.robot.state().error
            if err:
                self.lbl_status.setText(f"⚠ NOT-AUS angefordert — {err}")
            else:
                self.lbl_status.setText("NOT-AUS: Dämpfungsmodus aktiv")
        else:
            self.lbl_status.setText("NOT-AUS (nicht verbunden)")
        self.btn_control.setChecked(False)
        LOG.warning("NOT-AUS ausgelöst.")
        self._update_control_ui()

    # ------------------------------------------------------ Gelenkauswahl
    def _on_tree_select(self, item: QTreeWidgetItem | None, _prev) -> None:
        if item is None:
            return
        idx = item.data(0, Qt.UserRole)
        if idx is not None:
            self._select_joint(int(idx), from_tree=True)

    def _select_joint(self, index: int | None, from_tree: bool = False) -> None:
        self.selected = index
        self.view.set_selected(index)
        if index is not None:
            self.tabs.setCurrentWidget(self.joint_panel)
            self.joint_panel.show_joint(index, self._current_target(index))
            if not from_tree:
                self._sync_tree_selection(index)
        else:
            self.joint_panel.show_joint(None)
        self._update_control_ui()

    def _sync_tree_selection(self, index: int) -> None:
        self.tree.blockSignals(True)
        for t in range(self.tree.topLevelItemCount()):
            top = self.tree.topLevelItem(t)
            for c in range(top.childCount()):
                child = top.child(c)
                if child.data(0, Qt.UserRole) == index:
                    self.tree.setCurrentItem(child)
        self.tree.blockSignals(False)

    # -------------------------------------------------------- Sollwerte
    def _current_target(self, index: int) -> float:
        if self.robot is not None:
            return float(self.robot.state().targets[index])
        return float(self.preview_q[index])

    def _on_target_changed(self, index: int, q: float) -> None:
        self._send_target(index, q, sync_ui=False)

    def _send_target(self, index: int, q: float, sync_ui: bool) -> None:
        if self.robot is not None:
            self.robot.set_target(index, q)
        else:
            self.preview_q[index] = BY_INDEX[index].clamp(q)
        if sync_ui and self.selected == index:
            self.joint_panel.set_target_display(q)

    def _capture_actual(self, index: int) -> None:
        if self.robot is not None:
            q = float(self.robot.state().q[index])
        else:
            q = float(self.preview_q[index])
        self._send_target(index, q, sync_ui=True)

    # ------------------------------------------------------------- Ticker
    def _update_control_ui(self) -> None:
        connected = self.robot is not None
        if not connected:
            # Offline: Regler posieren das 3D-Modell
            self.joint_panel.set_slider_enabled(True)
            self.joint_panel.set_hint("Offline-Modus: Regler bewegen nur das 3D-Modell.")
            self.btn_control.setEnabled(False)
        else:
            active = self.robot.state().control_active
            self.btn_control.setEnabled(True)
            self.joint_panel.set_slider_enabled(active)
            self.joint_panel.set_hint(
                "" if active else
                "Verbunden. „Steuerung aktivieren“ drücken, um Gelenke zu bewegen.")
        # Slider/Spinbox auf den tatsächlichen Sollwert des Backends
        # zurücksetzen: enable_control()/emergency_damp() übernehmen die
        # Ist-Pose als Sollwert — ein stehengebliebener alter Sliderwert
        # würde sonst beim nächsten Schritt eine große Bewegung auslösen.
        if self.selected is not None:
            self.joint_panel.set_target_display(self._current_target(self.selected))

    def _tick(self) -> None:
        if self.robot is not None:
            state = self.robot.state()
            self.view.set_pose(state.q)
            if self.selected is not None:
                self.joint_panel.set_actual(float(state.q[self.selected]))
            if state.error:
                self.lbl_status.setText(f"⚠ {state.error}")
            elif state.control_active:
                self.lbl_status.setText(f"Verbunden: {self.robot.name} — Steuerung AKTIV")
            else:
                self.lbl_status.setText(f"Verbunden: {self.robot.name} — Dämpfung/passiv")
            if self.btn_control.isChecked() != state.control_active:
                self.btn_control.setChecked(state.control_active)
                self._update_control_ui()
        else:
            self.view.set_pose(self.preview_q)
            if self.selected is not None:
                self.joint_panel.set_actual(float(self.preview_q[self.selected]))

    def closeEvent(self, ev) -> None:
        self._disconnect()
        super().closeEvent(ev)
