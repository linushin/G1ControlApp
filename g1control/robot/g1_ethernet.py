"""Echte G1-Verbindung über Ethernet mit der offiziellen unitree_sdk2_python.

Ablauf entsprechend dem offiziellen Low-Level-Beispiel von Unitree
(``example/g1/low_level/g1_low_level_example.py``):

1. ``ChannelFactoryInitialize(domain_id, interface)`` — DDS über die
   angegebene Ethernet-Schnittstelle initialisieren.
2. ``rt/lowstate`` abonnieren (Ist-Zustand, ``mode_machine``).
3. Erst beim Aktivieren der Steuerung: über den ``MotionSwitcherClient``
   einen evtl. laufenden High-Level-Bewegungsdienst freigeben (sonst
   kollidieren die Befehle) und ``rt/lowcmd`` mit 500 Hz publizieren:
   PD-Positionsregelung pro Motor (mode=1, q, dq=0, kp, kd, tau=0) inkl. CRC.

Sicherheit:

* **Verbinden ist passiv**: Es wird nur ``rt/lowstate`` gelesen. Ein
  laufender High-Level-Dienst (z. B. Balance) bleibt unangetastet; erst
  ``enable_control`` gibt ihn frei und beginnt zu senden.
* Beim Aktivieren der Steuerung wird die Ist-Pose als Sollwert übernommen
  und gehalten; Slider-Änderungen werden mit begrenzter Geschwindigkeit
  (Slew-Rate) angefahren.
* ``disable_control``/``emergency_damp`` schalten auf reine Dämpfung
  (kp=0, kd>0) — der Roboter wird weich.
* **Watchdog**: Fällt ``rt/lowstate`` während aktiver Steuerung aus,
  schaltet die Regelschleife automatisch in die Dämpfung, statt blind
  weiter Positionsbefehle zu senden.
* Sollwerte werden immer auf die offiziellen Gelenkgrenzen begrenzt.
"""

from __future__ import annotations

import logging
import threading
import time

import numpy as np

from ..model.joints import KD, KP, NUM_JOINTS
from .interface import RobotInterface, RobotState

LOG = logging.getLogger(__name__)

# DDS kann pro Prozess nur einmal initialisiert werden — Interface merken.
_factory_iface: str | None = None
_factory_lock = threading.Lock()

MODE_PR = 0     # Pitch/Roll-Ansteuerung der Sprunggelenke (Serienmodus)


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
    DAMPING_KD = 1.5         # Dämpfung im passiven Modus
    STATE_TIMEOUT = 3.0      # s ohne LowState => Fehler / Watchdog-Dämpfung
    RELEASE_TIMEOUT = 10.0   # s für die Freigabe des High-Level-Dienstes

    def __init__(self, name: str, interface: str, domain_id: int = 0) -> None:
        super().__init__(name)
        self.interface = interface
        self.domain_id = domain_id
        self._running = False
        self._thread: threading.Thread | None = None
        self._publisher = None
        self._subscriber = None
        self._crc = None
        self._low_cmd = None
        self._mode_machine = 0
        self._last_state_time = 0.0
        self._cmd_q = np.zeros(NUM_JOINTS)      # aktuell gesendete Position
        self._damping = True                    # True => nur Dämpfung senden
        self._send_failed = False
        self._release_issued = False            # ReleaseMode je abgesetzt?
        # Empfangspuffer des DDS-Callbacks (nur vom Callback-Thread berührt).
        self._temp_scalar: bool | None = None
        self._rx_q = np.zeros(NUM_JOINTS)
        self._rx_dq = np.zeros(NUM_JOINTS)
        self._rx_tau = np.zeros(NUM_JOINTS)
        self._rx_temp = np.zeros(NUM_JOINTS)

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

        # Zustand einer evtl. früheren Verbindung verwerfen — sonst
        # "gelingt" die LowState-Wartschleife sofort am alten Zeitstempel.
        self._last_state_time = 0.0
        self._send_failed = False
        self._release_issued = False

        self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._subscriber.Init(self._on_low_state, 10)

        # Publisher nur vorbereiten — gesendet wird erst nach enable_control().
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

        # Verbinden ist bewusst passiv: kein ReleaseMode, kein rt/lowcmd.
        # Ein stehender Roboter bleibt unter seinem High-Level-Regler.
        with self._lock:
            self._state.connected = True
            self._state.error = ""
            self._cmd_q = self._state.q.copy()
            self._state.targets = self._state.q.copy()
            self._damping = True

    def _release_high_level_service(self) -> None:
        """High-Level-Bewegungsdienst freigeben; wirft bei Misserfolg.

        Wird erst beim Aktivieren der Steuerung aufgerufen, damit das
        bloße Verbinden einen stehenden Roboter nicht aus der Balance
        nimmt — und bei JEDER Aktivierung erneut geprüft, da der Dienst
        extern (Fernbedienung, Auto-Recovery) neu gestartet worden sein
        kann. Fehler werden nicht verschluckt: Solange die Freigabe nicht
        bestätigt ist, darf kein Positionsbefehl gesendet werden.
        """
        try:
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
                MotionSwitcherClient,
            )
        except ImportError:
            LOG.warning("SDK ohne MotionSwitcherClient — Freigabe des "
                        "High-Level-Dienstes kann nicht geprüft werden.")
            return  # ältere SDK-Version ohne MotionSwitcher
        msc = MotionSwitcherClient()
        msc.SetTimeout(5.0)
        msc.Init()
        deadline = time.time() + self.RELEASE_TIMEOUT
        while True:
            status, result = msc.CheckMode()
            if status != 0:
                raise RuntimeError(
                    f"MotionSwitcher CheckMode fehlgeschlagen (Status {status})."
                )
            name = result.get("name") if result else ""
            if not name:
                return
            if time.time() > deadline:
                raise RuntimeError(
                    f"High-Level-Dienst '{name}' ließ sich nicht freigeben — "
                    "Steuerung wird nicht aktiviert."
                )
            self._release_issued = True
            msc.ReleaseMode()
            time.sleep(1.0)

    def disconnect(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            # Vor dem Stopp kurz Dämpfung senden, damit der Roboter weich wird.
            self.disable_control()
            time.sleep(0.1)
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                # Referenz behalten: _start_loop darf keine zweite Schleife
                # neben einer festhängenden alten starten.
                LOG.error("Regelschleife beendet sich nicht binnen 2 s.")
            else:
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
        # Läuft mit 500 Hz: erst ohne Lock in eigene Puffer lesen, dann nur
        # für die Bulk-Zuweisung sperren (kurzer kritischer Abschnitt).
        ms = msg.motor_state
        if self._temp_scalar is None:
            self._temp_scalar = not hasattr(ms[0].temperature, "__len__")
        q, dq, tau, temp = self._rx_q, self._rx_dq, self._rx_tau, self._rx_temp
        if self._temp_scalar:
            for i in range(NUM_JOINTS):
                m = ms[i]
                q[i] = m.q
                dq[i] = m.dq
                tau[i] = m.tau_est
                temp[i] = m.temperature
        else:
            for i in range(NUM_JOINTS):
                m = ms[i]
                q[i] = m.q
                dq[i] = m.dq
                tau[i] = m.tau_est
                temp[i] = m.temperature[0]
        with self._lock:
            self._mode_machine = msg.mode_machine
            self._state.mode_machine = msg.mode_machine
            self._state.q[:] = q
            self._state.dq[:] = dq
            self._state.tau[:] = tau
            self._state.temperature[:] = temp
        # Erst nach dem Befüllen setzen: connect() wartet auf diesen Zeitstempel
        # und darf keinen leeren (Null-)Zustand als Pose übernehmen.
        self._last_state_time = time.time()

    # ------------------------------------------------------------------
    def state(self) -> RobotState:
        s = super().state()
        stale = self._last_state_time > 0 and (time.time() - self._last_state_time) > self.STATE_TIMEOUT
        if stale and not s.error:
            s.error = "Keine LowState-Daten (Verbindung unterbrochen?)"
        return s

    def enable_control(self) -> None:
        with self._lock:
            if not self._state.connected:
                raise RuntimeError("Nicht verbunden.")
        if (time.time() - self._last_state_time) > self.STATE_TIMEOUT:
            raise RuntimeError(
                "Keine aktuellen LowState-Daten — Verbindung prüfen, "
                "Steuerung wird nicht aktiviert."
            )
        try:
            self._release_high_level_service()
        except Exception:
            if self._release_issued:
                # ReleaseMode wurde bereits abgesetzt: Der High-Level-Dienst
                # ist evtl. schon gestoppt. Den Roboter nicht reglerlos
                # lassen — wenigstens Dämpfung streamen.
                with self._lock:
                    self._damping = True
                    self._state.error = ("Freigabe unklar — Dämpfung wird "
                                         "vorsorglich gesendet.")
                self._start_loop()
            raise
        with self._lock:
            self._cmd_q = self._state.q.copy()
            self._state.targets = self._state.q.copy()
            self._damping = False
            self._state.control_active = True
            self._state.error = ""
        self._start_loop()

    def disable_control(self) -> None:
        with self._lock:
            self._damping = True
            self._state.control_active = False

    def emergency_damp(self) -> bool:
        with self._lock:
            self._damping = True
            self._state.control_active = False
            self._state.targets = self._state.q.copy()
        # Dämpfung erreicht den Roboter nur über eine laufende Sendeschleife;
        # ohne je aktivierte Steuerung wurde nichts gesendet (und das ist
        # richtig so — der Roboter läuft dann noch unter eigener Regelung).
        return self._running and self._thread is not None and self._thread.is_alive()

    # ------------------------------------------------------------------
    def _start_loop(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            if self._running:
                return
            # Alte Schleife läuft noch aus (Join im disconnect lief ins
            # Timeout) — keine zweite parallel dazu starten.
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                raise RuntimeError(
                    "Die vorherige Regelschleife beendet sich nicht — "
                    "bitte die App neu starten."
                )
        self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    def _control_loop(self) -> None:
        dt = 1.0 / self.RATE_HZ
        next_t = time.perf_counter()
        while self._running:
            with self._lock:
                if not self._damping and (time.time() - self._last_state_time) > self.STATE_TIMEOUT:
                    # Watchdog: ohne aktuelles Feedback nicht blind
                    # weiterfahren — sofort in die Dämpfung.
                    self._damping = True
                    self._state.control_active = False
                    self._state.error = ("Keine LowState-Daten — Steuerung "
                                         "automatisch in Dämpfung geschaltet.")
                damping = self._damping
                if not damping:
                    self._cmd_q = self._slew_step(self._cmd_q, self._state.targets,
                                                  self.MAX_SPEED, dt)
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
                mc.kp = float(KP[i])
                mc.kd = float(KD[i])
        cmd.crc = self._crc.Crc(cmd)
        try:
            self._publisher.Write(cmd)
        except Exception as exc:
            self._send_failed = True
            with self._lock:
                # Eine bestehende Zustandsmeldung (z. B. vom Watchdog)
                # nicht überschreiben — sie ginge beim nächsten
                # erfolgreichen Senden mit verloren.
                if not self._state.error or self._state.error.startswith("Senden fehlgeschlagen"):
                    self._state.error = f"Senden fehlgeschlagen: {exc}"
        else:
            if self._send_failed:
                # Senden funktioniert wieder — veralteten Fehler löschen.
                self._send_failed = False
                with self._lock:
                    if self._state.error.startswith("Senden fehlgeschlagen"):
                        self._state.error = ""
