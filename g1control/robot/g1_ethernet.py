"""Echte G1-Verbindung über Ethernet mit der offiziellen unitree_sdk2_python.

Ablauf entsprechend dem offiziellen Low-Level-Beispiel von Unitree
(``example/g1/low_level/g1_low_level_example.py``):

1. ``ChannelFactoryInitialize(domain_id, interface)`` — DDS über die
   angegebene Ethernet-Schnittstelle initialisieren.
2. Über den ``MotionSwitcherClient`` einen evtl. laufenden
   High-Level-Bewegungsdienst freigeben (sonst kollidieren die Befehle).
3. ``rt/lowstate`` abonnieren (Ist-Zustand, ``mode_machine``).
4. ``rt/lowcmd`` mit 500 Hz publizieren: PD-Positionsregelung pro Motor
   (mode=1, q, dq=0, kp, kd, tau=0) inkl. CRC.

Sicherheit:

* Beim Aktivieren der Steuerung wird die Ist-Pose als Sollwert übernommen
  und gehalten; Slider-Änderungen werden mit begrenzter Geschwindigkeit
  (Slew-Rate) angefahren.
* ``disable_control``/``emergency_damp`` schalten auf reine Dämpfung
  (kp=0, kd>0) — der Roboter wird weich.
* Sollwerte werden immer auf die offiziellen Gelenkgrenzen begrenzt.
"""

from __future__ import annotations

import threading
import time

import numpy as np

from ..model.joints import JOINTS, NUM_JOINTS
from .interface import RobotInterface, RobotState

_LOWER = np.array([j.lower for j in JOINTS])
_UPPER = np.array([j.upper for j in JOINTS])
_KP = np.array([j.kp for j in JOINTS])
_KD = np.array([j.kd for j in JOINTS])

# DDS kann pro Prozess nur einmal initialisiert werden — Interface merken.
_factory_iface: str | None = None
_factory_lock = threading.Lock()

MODE_PR = 0     # Pitch/Roll-Ansteuerung der Sprunggelenke (Serienmodus)
MODE_MACHINE_UNSET = 0


class SdkNotInstalledError(RuntimeError):
    pass


def sdk_available() -> bool:
    try:
        import unitree_sdk2py  # noqa: F401
        return True
    except ImportError:
        return False


class G1EthernetRobot(RobotInterface):
    """Low-Level-Steuerung eines G1 über Ethernet (DDS)."""

    RATE_HZ = 500.0
    MAX_SPEED = 0.6          # rad/s — Verfahrgeschwindigkeit der Sollwerte
    DAMPING_KD = 1.5         # Dämpfung im passiven Modus
    STATE_TIMEOUT = 3.0      # s ohne LowState => Fehler

    def __init__(self, name: str, interface: str, domain_id: int = 0) -> None:
        super().__init__(name)
        self.interface = interface
        self.domain_id = domain_id
        self._lock = threading.Lock()
        self._state = RobotState()
        self._running = False
        self._thread: threading.Thread | None = None
        self._publisher = None
        self._subscriber = None
        self._crc = None
        self._low_cmd = None
        self._mode_machine = MODE_MACHINE_UNSET
        self._last_state_time = 0.0
        self._cmd_q = np.zeros(NUM_JOINTS)      # aktuell gesendete Position
        self._damping = True                    # True => nur Dämpfung senden

    # ------------------------------------------------------------------
    def connect(self) -> None:
        try:
            from unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize,
                ChannelPublisher,
                ChannelSubscriber,
            )
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
            from unitree_sdk2py.utils.crc import CRC
        except ImportError as exc:
            raise SdkNotInstalledError(
                "unitree_sdk2_python ist nicht installiert. "
                "Bitte ./install.sh ausführen (siehe README)."
            ) from exc

        global _factory_iface
        with _factory_lock:
            if _factory_iface is None:
                ChannelFactoryInitialize(self.domain_id, self.interface)
                _factory_iface = self.interface
            elif _factory_iface != self.interface:
                raise RuntimeError(
                    f"DDS wurde in diesem Prozess bereits für Schnittstelle "
                    f"'{_factory_iface}' initialisiert. Für eine andere "
                    f"Schnittstelle bitte die App neu starten."
                )

        self._crc = CRC()
        self._low_cmd = unitree_hg_msg_dds__LowCmd_()

        self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._subscriber.Init(self._on_low_state, 10)

        self._publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._publisher.Init()

        # Auf ersten LowState warten (Verbindungstest + mode_machine).
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if self._last_state_time > 0:
                break
            time.sleep(0.05)
        else:
            self._cleanup_channels()
            raise TimeoutError(
                f"Kein LowState vom Roboter über '{self.interface}' empfangen. "
                "Ethernet-Verbindung und IP-Konfiguration (192.168.123.x) prüfen."
            )

        self._release_high_level_service()

        with self._lock:
            self._state.connected = True
            self._state.error = ""
            self._cmd_q = self._state.q.copy()
            self._state.targets = self._state.q.copy()
        self._damping = True
        self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    def _release_high_level_service(self) -> None:
        """High-Level-Bewegungsdienst freigeben (offizielle Vorgehensweise)."""
        try:
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
                MotionSwitcherClient,
            )
        except ImportError:
            return  # ältere SDK-Version ohne MotionSwitcher
        try:
            msc = MotionSwitcherClient()
            msc.SetTimeout(5.0)
            msc.Init()
            status, result = msc.CheckMode()
            while result is not None and result.get("name"):
                msc.ReleaseMode()
                status, result = msc.CheckMode()
                time.sleep(1.0)
        except Exception:
            # Kein harter Fehler: z. B. wenn der Dienst bereits gestoppt ist.
            pass

    def disconnect(self) -> None:
        self.disable_control()
        time.sleep(0.1)
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._cleanup_channels()
        with self._lock:
            self._state.connected = False
            self._state.control_active = False

    def _cleanup_channels(self) -> None:
        for ch in (self._subscriber, self._publisher):
            try:
                if ch is not None:
                    ch.Close()
            except Exception:
                pass
        self._subscriber = None
        self._publisher = None

    # ------------------------------------------------------------------
    def _on_low_state(self, msg) -> None:
        self._last_state_time = time.time()
        with self._lock:
            self._mode_machine = msg.mode_machine
            self._state.mode_machine = msg.mode_machine
            for i in range(NUM_JOINTS):
                m = msg.motor_state[i]
                self._state.q[i] = m.q
                self._state.dq[i] = m.dq
                self._state.tau[i] = m.tau_est
                self._state.temperature[i] = m.temperature[0] if hasattr(m.temperature, "__len__") else m.temperature

    # ------------------------------------------------------------------
    def state(self) -> RobotState:
        with self._lock:
            s = self._state
            stale = self._last_state_time > 0 and (time.time() - self._last_state_time) > self.STATE_TIMEOUT
            return RobotState(
                connected=s.connected,
                control_active=s.control_active,
                q=s.q.copy(),
                dq=s.dq.copy(),
                tau=s.tau.copy(),
                temperature=s.temperature.copy(),
                targets=s.targets.copy(),
                mode_machine=s.mode_machine,
                error="Keine LowState-Daten (Verbindung unterbrochen?)" if stale else s.error,
            )

    def enable_control(self) -> None:
        with self._lock:
            if not self._state.connected:
                raise RuntimeError("Nicht verbunden.")
            self._cmd_q = self._state.q.copy()
            self._state.targets = self._state.q.copy()
            self._damping = False
            self._state.control_active = True

    def disable_control(self) -> None:
        with self._lock:
            self._damping = True
            self._state.control_active = False

    def set_target(self, joint_index: int, q: float) -> None:
        q = float(np.clip(q, _LOWER[joint_index], _UPPER[joint_index]))
        with self._lock:
            self._state.targets[joint_index] = q

    def emergency_damp(self) -> None:
        with self._lock:
            self._damping = True
            self._state.control_active = False
            self._state.targets = self._state.q.copy()

    # ------------------------------------------------------------------
    def _control_loop(self) -> None:
        dt = 1.0 / self.RATE_HZ
        next_t = time.perf_counter()
        while self._running:
            with self._lock:
                damping = self._damping
                if not damping:
                    delta = self._state.targets - self._cmd_q
                    step = np.clip(delta, -self.MAX_SPEED * dt, self.MAX_SPEED * dt)
                    self._cmd_q = np.clip(self._cmd_q + step, _LOWER, _UPPER)
                cmd_q = self._cmd_q.copy()
                mode_machine = self._mode_machine
            self._publish_cmd(cmd_q, damping, mode_machine)
            next_t += dt
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()

    def _publish_cmd(self, cmd_q: np.ndarray, damping: bool, mode_machine: int) -> None:
        cmd = self._low_cmd
        cmd.mode_pr = MODE_PR
        cmd.mode_machine = mode_machine
        for i in range(NUM_JOINTS):
            mc = cmd.motor_cmd[i]
            mc.mode = 1  # Motor aktiv
            mc.tau = 0.0
            mc.dq = 0.0
            if damping:
                mc.q = 0.0
                mc.kp = 0.0
                mc.kd = self.DAMPING_KD
            else:
                mc.q = float(cmd_q[i])
                mc.kp = float(_KP[i])
                mc.kd = float(_KD[i])
        cmd.crc = self._crc.Crc(cmd)
        try:
            self._publisher.Write(cmd)
        except Exception as exc:
            with self._lock:
                self._state.error = f"Senden fehlgeschlagen: {exc}"
