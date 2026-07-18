# G1 Control App

Desktop-App (Ubuntu 22.04) zur Steuerung eines **Unitree G1 Edu** über die
offizielle [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python).

* Rotierbares 3D-Modell des G1 — Gelenke direkt im Modell anklickbar
* Seitenmenü mit Schieberegler über den **vollen offiziellen Bewegungsbereich**
  jedes Gelenks (alle 29 Motoren/Aktuatoren)
* Verbindung über **Ethernet-Kabel** (DDS Low-Level-Control, `rt/lowcmd` / `rt/lowstate`)
* Verwaltung **mehrerer G1** über Roboterprofile
* **Handsteuerung als Platzhalter** — Schnittstelle für das spätere Handmodell
  (z. B. Dex3-1) ist bereits vorbereitet
* **Simulationsmodus** zum Testen ohne Hardware

> ⚠️ **Sicherheit:** Die App sendet Low-Level-Motorbefehle. Den Roboter für
> Gelenktests **aufhängen oder sicher lagern** (Debug-/Entwicklungsmodus laut
> Unitree-Handbuch), Umfeld freihalten und die Funk-Fernbedienung als
> zusätzlichen Not-Aus bereithalten. Der Button **NOT-AUS (Dämpfung)** schaltet
> alle Motoren sofort auf reine Dämpfung (kp = 0), sobald die App Low-Level-
> Befehle sendet (ab **Steuerung aktivieren**). Das **Verbinden ist passiv**
> (es wird nur gelesen); vorher sendet die App grundsätzlich keine Befehle —
> in dem Fall meldet der NOT-AUS-Button das ehrlich und der Roboter wird über
> die Funk-Fernbedienung gestoppt. Fällt das LowState-Feedback während aktiver
> Steuerung aus, schaltet ein **Watchdog** automatisch in die Dämpfung.

## Installation (Ubuntu 22.04)

```bash
./install.sh   # Systempakete, venv, PySide6/PyOpenGL/numpy, unitree_sdk2_python
./run.sh       # App starten
```

`install.sh` installiert die offizielle `unitree_sdk2_python` direkt aus dem
Unitree-Repository (nicht auf PyPI verfügbar); dabei wird auch
`cyclonedds 0.10.2` eingerichtet.

### Netzwerk einrichten (Ethernet-Direktverbindung)

1. G1 per Ethernet-Kabel mit dem PC verbinden.
2. Der PC-Schnittstelle eine statische IP im Roboternetz geben, z. B.:
   `IP 192.168.123.99`, Maske `255.255.255.0` (der G1 hat `192.168.123.164`).
3. Test: `ping 192.168.123.164`
4. In der App unter **Verwalten…** die verwendete Schnittstelle (z. B.
   `enp2s0`, per `ip addr` ermittelbar) im Roboterprofil eintragen.

## Bedienung

1. **Roboterprofil** wählen (oder über *Verwalten…* mehrere G1 anlegen —
   Name, Schnittstelle, DDS-Domain werden gespeichert).
2. **Verbinden** — die App wartet auf `rt/lowstate` und zeigt die aktuelle
   Ist-Pose im 3D-Modell. Das Verbinden ist rein passiv: Es werden keine
   Befehle gesendet, ein laufender High-Level-Dienst (z. B. Balance) bleibt
   aktiv.
3. Im 3D-Modell ein **Gelenk anklicken** (oder links in der Liste wählen) —
   rechts erscheint das Menü mit Schieberegler, Gradzahl-Eingabe und den
   offiziellen Grenzwerten. Drehen: linke Maustaste ziehen, Zoom: Mausrad,
   Verschieben: rechte Maustaste.
4. **Steuerung aktivieren** — ein laufender High-Level-Bewegungsdienst wird
   über den `MotionSwitcherClient` freigegeben (schlägt die Freigabe fehl,
   wird die Steuerung **nicht** aktiviert), die aktuelle Pose wird als
   Sollwert übernommen und gehalten (PD-Regelung, 500 Hz). Ab jetzt bewegt
   der Schieberegler das gewählte Gelenk; Sollwerte werden auf die
   Gelenkgrenzen begrenzt und mit maximal 0,6 rad/s angefahren
   (Slew-Rate-Begrenzung). Bei ausbleibendem `rt/lowstate` schaltet der
   Watchdog automatisch in die Dämpfung.
5. **NOT-AUS (Dämpfung)** stoppt die Regelung sofort (kp = 0, kd > 0).

Ohne Verbindung (Offline-Modus) bewegen die Regler nur das 3D-Modell —
praktisch zum Posieren und zum Kennenlernen der Freiheitsgrade. Der Button
**Simulation** verbindet mit einem simulierten G1 inklusive Bewegungsablauf.

## Die 29 Freiheitsgrade (offizielle Unitree-Daten)

Indizes, Namen und Winkelgrenzen entsprechen dem offiziellen G1 Developer
Guide bzw. der offiziellen `g1_29dof.urdf` von Unitree Robotics
(x nach vorn, y nach links, z nach oben; Winkel in rad):

| Idx | Gelenk | Grenzen [rad] | max. Nm |
|----:|--------|---------------|--------:|
| 0 | left_hip_pitch | −2.5307 … +2.8798 | 88 |
| 1 | left_hip_roll | −0.5236 … +2.9671 | 88 |
| 2 | left_hip_yaw | −2.7576 … +2.7576 | 88 |
| 3 | left_knee | −0.0873 … +2.8798 | 139 |
| 4 | left_ankle_pitch | −0.8727 … +0.5236 | 35 |
| 5 | left_ankle_roll | −0.2618 … +0.2618 | 35 |
| 6 | right_hip_pitch | −2.5307 … +2.8798 | 88 |
| 7 | right_hip_roll | −2.9671 … +0.5236 | 88 |
| 8 | right_hip_yaw | −2.7576 … +2.7576 | 88 |
| 9 | right_knee | −0.0873 … +2.8798 | 139 |
| 10 | right_ankle_pitch | −0.8727 … +0.5236 | 35 |
| 11 | right_ankle_roll | −0.2618 … +0.2618 | 35 |
| 12 | waist_yaw | −2.618 … +2.618 | 88 |
| 13 | waist_roll¹ | −0.52 … +0.52 | 35 |
| 14 | waist_pitch¹ | −0.52 … +0.52 | 35 |
| 15 | left_shoulder_pitch | −3.0892 … +2.6704 | 25 |
| 16 | left_shoulder_roll | −1.5882 … +2.2515 | 25 |
| 17 | left_shoulder_yaw | −2.618 … +2.618 | 25 |
| 18 | left_elbow | −1.0472 … +2.0944 | 25 |
| 19 | left_wrist_roll | −1.9722 … +1.9722 | 25 |
| 20 | left_wrist_pitch¹ | −1.6144 … +1.6144 | 5 |
| 21 | left_wrist_yaw¹ | −1.6144 … +1.6144 | 5 |
| 22 | right_shoulder_pitch | −3.0892 … +2.6704 | 25 |
| 23 | right_shoulder_roll | −2.2515 … +1.5882 | 25 |
| 24 | right_shoulder_yaw | −2.618 … +2.618 | 25 |
| 25 | right_elbow | −1.0472 … +2.0944 | 25 |
| 26 | right_wrist_roll | −1.9722 … +1.9722 | 25 |
| 27 | right_wrist_pitch¹ | −1.6144 … +1.6144 | 5 |
| 28 | right_wrist_yaw¹ | −1.6144 … +1.6144 | 5 |

¹ Bei der 23-DOF-Variante gesperrt; die App ist für den G1 Edu (29 DOF)
ausgelegt. Die Sprunggelenke werden im **PR-Modus** (Pitch/Roll,
`mode_pr = 0`) angesteuert.

## Architektur

```
g1control/
├── model/
│   ├── joints.py       # 29 Gelenke: Indizes, offizielle Limits, Kp/Kd
│   └── kinematics.py   # Gelenkbaum aus der offiziellen URDF + Vorwärtskinematik
├── robot/
│   ├── interface.py    # abstrakte Roboterschnittstelle (RobotInterface)
│   ├── g1_ethernet.py  # echte Verbindung: DDS Low-Level via unitree_sdk2_python
│   ├── mock.py         # Simulations-Backend ohne Hardware
│   ├── hands.py        # HandInterface + PlaceholderHand (Handmodell folgt)
│   └── profiles.py     # Profile mehrerer G1 (~/.config/g1control/robots.json)
└── ui/
    ├── main_window.py  # Toolbar, Gelenkliste, 3D-Ansicht, Seitenmenü
    ├── robot_view.py   # rotierbares 3D-Modell, Color-Picking der Gelenke
    ├── joint_panel.py  # Schieberegler über den vollen Gelenkbereich
    ├── hand_panel.py   # Platzhalter-Handsteuerung
    └── connection_dialog.py  # Verwaltung mehrerer Roboterprofile
```

### Ansteuerung (Low-Level, wie im offiziellen Unitree-Beispiel)

* `ChannelFactoryInitialize(domain, interface)` auf der Ethernet-Schnittstelle
* `rt/lowstate` abonnieren (Ist-Winkel, `mode_machine`)
* Erst bei **Steuerung aktivieren**: `MotionSwitcherClient.ReleaseMode()` —
  High-Level-Dienst freigeben (mit Timeout und Fehlerprüfung)
* `rt/lowcmd` mit 500 Hz: pro Motor `mode=1`, `q`, `dq=0`, `tau=0` sowie
  Kp/Kd aus dem offiziellen `g1_low_level_example`, CRC-gesichert

### Mehrere Roboter

Profile lassen sich beliebig anlegen; verbunden wird jeweils der aktive
Roboter. Hinweis: Die DDS-Schicht der SDK kann pro Prozess nur **eine**
Netzwerkschnittstelle initialisieren — für einen Wechsel der Schnittstelle
die App neu starten. Gleichzeitige Verbindungen zu mehreren G1 sind als
späterer Ausbauschritt vorgesehen (die Schnittstelle `RobotInterface` ist
dafür bereits ausgelegt).

### Hände (Platzhalter)

`robot/hands.py` definiert `HandInterface` (normierte Freiheitsgrade 0…1,
Öffnen/Schließen). Die UI bedient ausschließlich diese Schnittstelle. Sobald
das konkrete Handmodell feststeht (z. B. Dex3-1, DDS-Topics
`rt/dex3/<seite>/cmd`), genügt eine neue Implementierung der Schnittstelle —
UI und App-Logik bleiben unverändert.
