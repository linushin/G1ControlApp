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
* **Watchdog**: Fällt ``rt/lowstate`` während aktiver Steuerung aus oder
  wirft die Regelschleife eine Ausnahme, wird automatisch in die Dämpfung
  geschaltet, statt blind Positionsbefehle zu senden.
* Sollwerte werden immer auf die offiziellen Gelenkgrenzen begrenzt;
  NaN/Inf werden an mehreren Stellen abgefangen und erreichen den
  Roboter nicht.
* Alle Zeitvergleiche (Watchdog, Frische-Gates) laufen auf
  ``time.monotonic`` — ein NTP-Sprung kann den Watchdog nicht blenden.
"""

from __future__ import annotations

import logging
import threading
import time

import numpy as np

from ..model.joints import KD, KP, LOWER, NUM_JOINTS, UPPER
from .interface import RobotInterface, RobotState

LOG = logging.getLogger(__name__)

# DDS kann pro Prozess nur einmal initialisiert werden — Schnittstelle UND
# Domain merken: Ein Profilwechsel auf eine andere Domain darf nicht
# stillschweigend den Roboter der alten Domain kommandieren.
_factory_key: tuple[str, int] | None = None
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
    SNAPSHOT_MAX_AGE = 1.0   # s: max. Alter der Pose, die als Sollwert übernommen wird
    RELEASE_TIMEOUT = 10.0   # s für die Freigabe des High-Level-Dienstes
    RELEASE_POLL_S = 0.2     # Poll-Intervall der Freigabe (kurz halten: in
                             # dieser Zeit ist der Roboter ggf. reglerlos)

    #: Vor der ersten Aktivierung am echten Roboter bestätigt die UI die
    #: Sicherheitshinweise (Balance-Dienst wird beendet, Roboter aufhängen).
    NEEDS_ENABLE_CONFIRMATION = True

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
        self._last_state_time = 0.0             # time.monotonic()-Basis
        self._cmd_q = np.zeros(NUM_JOINTS)      # aktuell gesendete Position
        self._damping = True                    # True => nur Dämpfung senden
        self._send_error = ""                   # letzter Write-Fehler ("" = ok)
        self._release_issued = False            # ReleaseMode in DIESEM Versuch?
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

        global _factory_key
        with _factory_lock:
            key = (self.interface, self.domain_id)
            if _factory_key is None:
                ChannelFactoryInitialize(self.domain_id, self.interface)
                _factory_key = key
            elif _factory_key != key:
                raise RuntimeError(
                    f"DDS wurde in diesem Prozess bereits für Schnittstelle "
                    f"'{_factory_key[0]}' (Domain {_factory_key[1]}) "
                    f"initialisiert. Für eine andere Schnittstelle oder "
                    f"Domain bitte die App neu starten."
                )

        self._crc = CRC()
        self._low_cmd = unitree_hg_msg_dds__LowCmd_()

        # Zustand einer evtl. früheren Verbindung verwerfen — sonst
        # "gelingt" die LowState-Wartschleife sofort am alten Zeitstempel.
        self._last_state_time = 0.0
        self._send_error = ""
        self._release_issued = False
        self._temp_scalar = None

        self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._subscriber.Init(self._on_low_state, 10)

        # Publisher nur vorbereiten — gesendet wird erst nach enable_control().
        self._publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._publisher.Init()

        # Auf ersten LowState warten (Verbindungstest + mode_machine).
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
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
            q = self._state.q.copy()
            self._cmd_q = q
            self._state.targets = np.clip(q, LOWER, UPPER)
            self._damping = True

    def _release_high_level_service(self) -> None:
        """High-Level-Bewegungsdienst freigeben; wirft bei Misserfolg.

        Wird erst beim Aktivieren der Steuerung aufgerufen, damit das
        bloße Verbinden einen stehenden Roboter nicht aus der Balance
        nimmt — und bei JEDER Aktivierung erneut geprüft, da der Dienst
        extern (Fernbedienung, Auto-Recovery) neu gestartet worden sein
        kann. Fehler werden nicht verschluckt: Solange die Freigabe nicht
        bestätigt ist, darf kein Positionsbefehl gesendet werden.

        ``_release_issued`` gilt nur für DIESEN Versuch (wird hier
        zurückgesetzt) — der Dämpfungs-Fallback in ``enable_control``
        darf nicht auf einem Flag aus einem früheren Versuch beruhen.
        """
        self._release_issued = False
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
        deadline = time.monotonic() + self.RELEASE_TIMEOUT
        while True:
            status, result = msc.CheckMode()
            if status != 0:
                raise RuntimeError(
                    f"MotionSwitcher CheckMode fehlgeschlagen (Status {status})."
                )
            name = result.get("name") if result else ""
            if not name:
                return
            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"High-Level-Dienst '{name}' ließ sich nicht freigeben — "
                    "Steuerung wird nicht aktiviert."
                )
            self._release_issued = True
            msc.ReleaseMode()
            # Kurz halten: Ab dem ReleaseMode ist der Roboter ggf. ohne
            # Regler, bis enable_control die Sendeschleife startet.
            time.sleep(self.RELEASE_POLL_S)

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
        self._last_state_time = time.monotonic()

    # ------------------------------------------------------------------
    def state(self) -> RobotState:
        s = super().state()
        # Sendefehler getrennt führen und ANHÄNGEN statt überschreiben:
        # Eine Watchdog-Meldung ("… in Dämpfung geschaltet") darf einen
        # gleichzeitigen Sendefehler nicht verdecken — sonst behauptet die
        # Anzeige Dämpfung, während nichts beim Roboter ankommt.
        if self._send_error:
            s.error = f"{s.error} | {self._send_error}" if s.error else self._send_error
        stale = self._last_state_time > 0 and (time.monotonic() - self._last_state_time) > self.STATE_TIMEOUT
        if stale and not s.error:
            s.error = "Keine LowState-Daten (Verbindung unterbrochen?)"
        return s

    def enable_control(self) -> None:
        with self._lock:
            if not self._state.connected:
                raise RuntimeError("Nicht verbunden.")
        if (time.monotonic() - self._last_state_time) > self.STATE_TIMEOUT:
            raise RuntimeError(
                "Keine aktuellen LowState-Daten — Verbindung prüfen, "
                "Steuerung wird nicht aktiviert."
            )
        try:
            self._release_high_level_service()
        except Exception:
            if self._release_issued:
                # ReleaseMode wurde in DIESEM Versuch abgesetzt: Der
                # High-Level-Dienst ist evtl. schon gestoppt. Den Roboter
                # nicht reglerlos lassen — wenigstens Dämpfung streamen.
                # Die Erfolgsmeldung erst NACH erfolgreichem Schleifenstart
                # setzen, sonst behauptet der Status ein Streaming, das es
                # nicht gibt.
                with self._lock:
                    self._damping = True
                try:
                    self._start_loop()
                except Exception:
                    LOG.exception("Dämpfungs-Fallback: Schleife startet nicht")
                    with self._lock:
                        if not self._state.error:
                            self._state.error = (
                                "Freigabe unklar und Sendeschleife startet "
                                "nicht — bitte App neu starten.")
                else:
                    with self._lock:
                        self._state.error = ("Freigabe unklar — Dämpfung "
                                             "wird vorsorglich gesendet.")
            raise
        # Nach der (ggf. langen) Freigabe: Frische erneut prüfen. Der erste
        # steife Frame darf keine veraltete Pose anspringen — der Watchdog
        # greift erst ab STATE_TIMEOUT und würde bis dahin schweigen.
        if (time.monotonic() - self._last_state_time) > self.SNAPSHOT_MAX_AGE:
            with self._lock:
                self._damping = True
                self._state.error = ("LowState während der Freigabe "
                                     "veraltet — Dämpfung wird gesendet, "
                                     "Steuerung nicht aktiviert.")
            self._start_loop()
            raise RuntimeError(
                "Keine aktuellen LowState-Daten nach der Freigabe — "
                "Steuerung wird nicht aktiviert."
            )
        # Schleife ZUERST starten (beginnt in Dämpfung): Scheitert der
        # Start, bleibt control_active korrekt False — und die Lücke
        # zwischen Freigabe und erstem gesendeten Frame bleibt minimal.
        self._start_loop()
        with self._lock:
            q = self._state.q.copy()
            if not np.all(np.isfinite(q)):
                self._damping = True
                self._state.error = ("Ungültige Gelenkdaten (NaN/Inf) vom "
                                     "Roboter — Steuerung nicht aktiviert.")
                raise RuntimeError(
                    "Ungültige Gelenkdaten (NaN/Inf) vom Roboter — "
                    "Steuerung wird nicht aktiviert."
                )
            # Ist-Pose halten (auch wenn sie minimal außerhalb der Tabelle
            # liegt — kein Positionssprung!), Ziele aber immer geklemmt:
            # _slew_step führt die Pose dann ratenbegrenzt in die Grenzen.
            self._cmd_q = q
            self._state.targets = np.clip(q, LOWER, UPPER)
            self._damping = False
            self._state.control_active = True
            self._state.error = ""
            self._send_error = ""

    def disable_control(self) -> None:
        with self._lock:
            self._damping = True
            self._state.control_active = False

    def emergency_damp(self) -> bool:
        with self._lock:
            self._damping = True
            self._state.control_active = False
            self._state.targets = self._state.q.copy()
        # Dämpfung erreicht den Roboter nur über eine laufende Sendeschleife,
        # deren Writes auch ankommen; ohne je aktivierte Steuerung wurde
        # nichts gesendet (und das ist richtig so — der Roboter läuft dann
        # noch unter eigener Regelung).
        return (self._running and self._thread is not None
                and self._thread.is_alive() and not self._send_error)

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
        error_logged = False
        while self._running:
            # Kein stiller Thread-Tod: Jede Ausnahme im Schleifenkörper
            # schaltet auf Dämpfung und die Schleife läuft weiter — sonst
            # zeigt die UI "AKTIV", während gar nichts mehr gesendet wird.
            try:
                with self._lock:
                    if not self._damping and (time.monotonic() - self._last_state_time) > self.STATE_TIMEOUT:
                        # Watchdog: ohne aktuelles Feedback nicht blind
                        # weiterfahren — sofort in die Dämpfung.
                        self._damping = True
                        self._state.control_active = False
                        self._state.error = ("Keine LowState-Daten — Steuerung "
                                             "automatisch in Dämpfung geschaltet.")
                    if not self._damping:
                        cmd = self._slew_step(self._cmd_q, self._state.targets,
                                              self.MAX_SPEED, dt)
                        if not np.all(np.isfinite(cmd)):
                            # Letztes Sicherheitsnetz: NaN/Inf niemals senden.
                            self._damping = True
                            self._state.control_active = False
                            self._state.error = ("Ungültige Sollwerte (NaN/Inf) "
                                                 "— Steuerung automatisch in "
                                                 "Dämpfung geschaltet.")
                        else:
                            self._cmd_q = cmd
                    damping = self._damping
                    cmd_q = self._cmd_q.copy()
                    mode_machine = self._mode_machine
                self._publish_cmd(cmd_q, damping, mode_machine)
            except Exception as exc:
                if not error_logged:
                    # Nur einmal loggen — bei 500 Hz würde ein anhaltender
                    # Fehler sonst das Log fluten.
                    LOG.exception("Fehler in der Regelschleife")
                    error_logged = True
                with self._lock:
                    self._damping = True
                    self._state.control_active = False
                    if not self._state.error:
                        self._state.error = f"Fehler in der Regelschleife: {exc}"
            else:
                error_logged = False
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
            self._send_error = f"Senden fehlgeschlagen: {exc}"
        else:
            self._send_error = ""
