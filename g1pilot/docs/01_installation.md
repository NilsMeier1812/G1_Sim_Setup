# Installation & Ersteinrichtung

Richtet sich an: alle Anwender. Ziel: von einem frischen Rechner zu einer
laufenden Simulation.

## Überblick

G1Pilot läuft komplett in Docker. Auf dem Host wird nur Docker, Git und ein
X11-Display gebraucht — der gesamte ROS-2-/MuJoCo-Stack steckt in den
Container-Images. Das Repository ist eigenständig (keine Git-Submodule): die
Unitree-Abhängigkeiten `unitree_mujoco`, `unitree_ros2`, `unitree_sdk2_python`
liegen mit im Baum.

Zwei Betriebsarten:

- **Simulation** — MuJoCo-Physik statt echtem Roboter, läuft auf jedem
  halbwegs aktuellen Rechner, keine GPU nötig.
- **Echter Roboter** — siehe zusätzlich
  [70_echtroboter_anleitung.md](70_echtroboter_anleitung.md), bevor der Stack
  gegen Hardware gestartet wird.

## Voraussetzungen

- Linux (getestet auf Ubuntu 22.04 / 24.04) oder Windows 10/11 mit WSL2.
- ≥ 8 GB RAM, ~10 GB freier Plattenplatz für die Docker-Images.
- Ein laufendes X11-Display (für RViz und die MuJoCo-/Teleop-Fenster).
  Unter Wayland hilft in der Regel `xhost` über den XWayland-Layer; über SSH
  mit `ssh -X` verbinden.
- Keine GPU/CUDA nötig — die Simulation ist rein CPU-basiert.

## Schritt für Schritt (Linux)

**1. System-Pakete**

```bash
sudo apt update
sudo apt install -y git x11-xserver-utils ca-certificates curl \
  python3-tk python3-venv
```

`python3-tk` und `python3-venv` laufen auf dem **Host**, nicht im Container:

- `python3-tk` — grafisches Startmenü von `./start.sh`. Fehlt es, startet
  ohne Fehlermeldung das Text-Menü.
- `python3-venv` — virtualenv für den Umgebungs-Builder / Scene-Editor
  (Schritt 5b). Ohne das Paket bricht `python3 -m venv` ab
  (`No module named 'ensurepip'`) und hinterlässt ein kaputtes `.venv`
  ohne `pip`.

**2. Docker Engine + Compose-Plugin** (offizielles Docker-Repository)

```bash
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
  sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin
```

**3. Docker ohne `sudo` nutzbar machen**

```bash
sudo usermod -aG docker $USER
newgrp docker          # oder: ab-/wieder anmelden
docker run --rm hello-world   # muss ohne sudo durchlaufen
```

**4. Repository klonen**

```bash
git clone https://github.com/nilsmeier1812/g1simrepo.git
cd g1simrepo/g1pilot
```

**5. Images bauen** (erstmalig, ca. 8 Minuten; lädt ROS 2 Humble, MuJoCo,
Pinocchio usw.)

```bash
make build-sim         # baut g1pilot-sim:v1.1.0 + g1pilot-mujoco:v1.0
```

**5b. Umgebungs-Builder / Scene-Editor einrichten** (einmalig, braucht
Internet für PyPI)

`start.sh` baut beim Auswählen einer Umgebung die Szene auf dem Host mit
`unitree_mujoco/scene_editor/build_env_scene.py`. Das Skript nutzt dafür
`trimesh` + `vhacdx` aus `scene_editor/.venv` (zerlegt Mesh-Hindernisse wie
den Arbeitsplatz in konvexe Teil-Hüllen). Dasselbe venv braucht der
Scene-Editor („Umgebungen bearbeiten").

**Python-Version:** `mujoco-scene-editor` braucht Python **3.10–3.12**
(Ubuntu 22.04: 3.10, 24.04: 3.12, beide ok). Ubuntu 26.04 bringt nur 3.14
mit. Dann ein 3.12 ohne sudo per [uv](https://docs.astral.sh/uv/)
installieren; `setup.sh` findet es automatisch:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
~/.local/bin/uv python install 3.12
```

```bash
cd ../unitree_mujoco/scene_editor
rm -rf .venv           # nur nötig, falls ein altes/kaputtes .venv existiert
./setup.sh             # legt .venv an, installiert requirements.txt
.venv/bin/python -c "import trimesh, vhacdx; print('ok')"   # muss 'ok' ausgeben
cd ../../g1pilot
```

Fehlen die Pakete, gibt es **keine sichtbare Fehlermeldung**: die Szene wird
mit nur einer konvexen Hülle pro Mesh gebaut, und Greif-Objekte werden beim
Start weggeschleudert (ständige `[scene-reset]`-Meldungen im Log). Ein `.venv`
nach dem Verschieben/Kopieren des Repos immer neu anlegen — virtualenvs
enthalten absolute Pfade. Details:
[unitree_mujoco/scene_editor/README.md](../../unitree_mujoco/scene_editor/README.md).

**6. Starten**

```bash
make sim               # baut bei Bedarf nach und startet den Stack
```

Beim ersten `make sim` wird zusätzlich `xhost +local:docker` gesetzt, damit
die Container auf das Display zugreifen dürfen. Erscheinen das MuJoCo-Fenster
und der Streamdeck, steht die Umgebung. Der Roboter steht dabei zunächst nur
— siehe [30_loco_anleitung.md](30_loco_anleitung.md) für die Bedienung.

## Windows (WSL2)

Läuft auf Windows, aber **innerhalb von WSL2**, nicht über „Docker Desktop für
Windows" pur. Zwei Dinge im Setup sind Linux-spezifisch: `network_mode: host`
(trägt die DDS-Kommunikation der Container über `lo`) und die X11-GUIs (RViz
+ MuJoCo-Viewer). Beides funktioniert in WSL2 sauber, in Docker Desktop pur
dagegen nicht zuverlässig. GPU/CUDA wird nicht gebraucht.

Empfohlen: Windows 11 (WSLg für die GUIs ist eingebaut).

```powershell
# In PowerShell: WSL2 + Ubuntu 24.04 installieren (ggf. danach neu starten)
wsl --install -d Ubuntu-24.04
```

Bewusst **Ubuntu-24.04** und nicht `Ubuntu`: `Ubuntu` ist immer die neueste
Version (26.04, Python 3.14) – damit laesst sich der Scene Editor nicht
einrichten (braucht Python < 3.13). Gestartet wird sie danach mit
`wsl -d Ubuntu-24.04` bzw. ueber „Ubuntu 24.04" im Startmenue.

Ist **Docker Desktop** installiert: es haengt sich standardmaessig in die
*Standard*-WSL-Distribution ein und kollidiert dort mit der nativen
Docker-Engine. Entweder Ubuntu nicht zur Standard-Distribution machen oder in
Docker Desktop unter *Settings → Resources → WSL integration* die Integration
fuer Ubuntu-24.04 ausschalten.

Danach im Ubuntu-Terminal (WSL) weiter — ab hier identisch zur
Linux-Anleitung oben:

**Vorher: Netzwerk prüfen.** Mit aktivem VPN (z. B. eduVPN an der FAU) kommt
WSL2 im Standard-Netzwerkmodus oft nicht ins Internet — siehe
[WSL2-Netzwerk](#wsl2-netzwerk-vpn-uni-wlan-roboter) unten. Test:

```bash
curl -sI https://github.com | head -1    # erwartet: HTTP/2 200
```

```bash
# System-Pakete inkl. python3-tk + python3-venv (Schritt 1 oben).
# Docker-Engine NATIV in der WSL-Distro installieren (Schritte 1-3 oben).
# Native Engine statt Docker-Desktop-Integration, damit Host-Networking
# ohne Tricks funktioniert.
sudo service docker start

# Repo INS WSL-Dateisystem klonen (NICHT nach /mnt/c/... — das ist langsam)
cd ~
git clone https://github.com/nilsmeier1812/g1simrepo.git
cd g1simrepo/g1pilot
make build-sim
# Umgebungs-Builder einrichten (Schritt 5b oben)
(cd ../unitree_mujoco/scene_editor && ./setup.sh)
make sim
```

WSLg setzt `DISPLAY` automatisch und stellt den X11-Socket bereit — das
MuJoCo-Fenster und RViz öffnen sich direkt auf dem Windows-Desktop.

Windows 10: geht ebenfalls über WSL2, aber WSLg ist nicht in jeder Version
dabei — dann einen X-Server (VcXsrv/X410) starten und `DISPLAY` von Hand
setzen. Docker Desktop statt nativer Engine ist möglich, `network_mode: host`
ist dort aber nur als (zu aktivierendes) Beta-Feature neuerer Versionen
verfügbar.

### WSL2-Netzwerk (VPN, Uni-WLAN, Roboter)

**Problem.** Im Standardmodus (NAT) hat WSL2 ein eigenes virtuelles Netz
(`172.x.x.x`) hinter Windows. Der Datenverkehr läuft dabei oft **am
VPN-Tunnel vorbei** direkt ins physische Netz. Im FAU-WLAN, das ohne VPN nur
FAU-Server erreicht, sieht das so aus:

| Ziel | Windows | WSL2 (NAT) |
|---|---|---|
| FAU-Server (`ftp.fau.de`, `gitlab.cs.fau.de`, …) | ✅ | ✅ |
| Alles andere (`archive.ubuntu.com`, PyPI, GitHub, Docker Hub, `packages.ros.org`) | ✅ | ❌ Timeout |

Betroffen sind damit `apt install`, `pip install` (auch `setup.sh`),
`git clone` von GitHub, `docker pull` und `make build-sim`. `apt` meldet dann
nur `Ign:` und bricht ab, `curl` läuft in einen Timeout. Außerdem liegt WSL
im NAT-Modus nicht im selben Netz wie ein per Ethernet angeschlossener
Roboter (`192.168.123.x`), und DDS-Multicast kommt nicht durch.

Kurz-Diagnose:

```bash
# in WSL
curl -4 -sS -m 8 -o /dev/null -w '%{http_code}\n' https://github.com   # 000 = blockiert
curl -4 -sS -m 8 -o /dev/null -w '%{http_code}\n' https://ftp.fau.de   # 200 = nur Uni-Netz geht
```

```powershell
# in PowerShell: kommt Windows selbst durch?
Test-NetConnection github.com -Port 443
```

Windows kommt durch, WSL nicht? Dann ist es dieses Problem.

**Lösung: Mirrored Networking** (Windows 11 22H2+, aktuelles WSL). WSL nutzt
dann direkt die Netzwerkadapter von Windows, inklusive VPN-Tunnel und
Roboter-Ethernet. Datei `C:\Users\<name>\.wslconfig` anlegen bzw. ergänzen:

```ini
[wsl2]
networkingMode=mirrored
dnsTunneling=true
autoProxy=true
```

Danach in PowerShell WSL neu starten und prüfen:

```powershell
wsl --shutdown
```

```bash
# wieder in WSL
curl -sI https://github.com | head -1    # erwartet: HTTP/2 200
```

Für DDS vom echten Roboter blockiert zusätzlich die Hyper-V-Firewall
eingehende Pakete. Freigeben in einer **Administrator**-PowerShell:

```powershell
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```

(Für den Echtbetrieb bleibt trotzdem ein nativer Linux-PC empfohlen, siehe
[70_echtroboter_anleitung.md](70_echtroboter_anleitung.md).)

**Wenn Mirrored Mode nicht hilft:**

- **apt:** auf den FAU-Mirror umstellen, der auch ohne VPN erreichbar ist:

  ```bash
  sudo cp /etc/apt/sources.list.d/ubuntu.sources ~/ubuntu.sources.bak
  sudo sed -i -e 's|http://archive.ubuntu.com/ubuntu/|http://ftp.fau.de/ubuntu/|' \
              -e 's|http://security.ubuntu.com/ubuntu/|http://ftp.fau.de/ubuntu/|' \
              /etc/apt/sources.list.d/ubuntu.sources
  sudo apt update
  ```

  Zurück: `sudo cp ~/ubuntu.sources.bak /etc/apt/sources.list.d/ubuntu.sources`.
- **pip / Docker Hub / GitHub:** Dafür gibt es keinen FAU-Mirror. Einmalig in
  einem freien Netz (eduroam, LAN, Handy-Hotspot, zu Hause) `make build-sim`
  und `scene_editor/setup.sh` ausführen. Danach braucht die Simulation kein
  Internet mehr.
- **Proxy:** Falls das Institut einen HTTP-Proxy anbietet, diesen für apt
  (`/etc/apt/apt.conf.d/`), pip (`HTTPS_PROXY`) und Docker
  (`/etc/systemd/system/docker.service.d/`) eintragen.

### Nur Umgebungen bauen: Scene Editor mit Docker Desktop

Für den Scene Editor (Umgebungen bauen, CAD/STEP importieren) reicht
**Docker Desktop** – er braucht weder Host-Networking noch X11, die
Oberfläche läuft im Browser. In PowerShell im Ordner `g1pilot/`:

```powershell
docker compose --profile editor build scene-editor         # einmalig, ~5 min
docker compose --profile editor run --rm --service-ports scene-editor
# -> Menü von launch.sh; Editor im Browser: http://127.0.0.1:8080
```

Details (CAD-Dateien importieren, Argumente für `launch.sh`) in
`unitree_mujoco/scene_editor/README.md`, Abschnitt „Setup unter Windows".

## Starten im Alltag

Der einfachste Einstieg ist `./start.sh`. Ohne Argumente öffnet sich ein
grafisches Startmenü (`g1_gui.py`, Tkinter): drei Karten — *Simulation
starten*, *Echten Roboter starten*, *Umgebungen bearbeiten*. Alles läuft in
einem Fenster; Menü, Optionsseiten und Log-Ansicht werden ausgetauscht.
Startet man einen Stack, erscheint dessen Docker-Ausgabe live im Fenster mit
einem Stop-Button. Über *‹ Menü* geht man zurück, ohne den Stack zu beenden —
er taucht unter *Laufende Prozesse* wieder auf. Fehlt Tkinter oder ein
Display, fällt `start.sh` automatisch auf ein klassisches Text-Menü zurück
(erzwingbar mit `--menu` oder `G1_NO_GUI=1`).

```bash
./start.sh            # grafisches Startmenü (Standard)
./start.sh --menu     # klassisches Text-Menü
# Nicht-interaktiv (Sim, Defaults/Env-Overrides):
USE_RVIZ=true ./start.sh --yes
# Nicht-interaktiv (Real, erfordert explizite Bestätigung):
G1_MODE=real ROBOT_INTERFACE=enp3s0 G1_REAL_CONFIRM=1 ./start.sh --yes
```

Alternativ direkt über `make` bzw. `docker compose`:

| Befehl | Wirkung |
|---|---|
| `make sim` | Stack im Vordergrund starten (Ctrl-C stoppt) |
| `make sim-bg` | Stack im Hintergrund |
| `make real ROBOT_INTERFACE=<nic>` | Echten Roboter starten (schlank: Arme + Hände + Loco) |
| `make real-full ROBOT_INTERFACE=<nic>` | Echten Roboter mit Livox/MOLA/Navigation starten |
| `make stop` | Stack stoppen |
| `make logs` / `make status` | Logs folgen / Container-Status |
| `make shell-sim` / `make shell-mujoco` / `make shell-real` | Shell im jeweiligen Container |
| `make clean` | Container + Images entfernen |

Jedes `make sim`/`make real`/`make real-full` baut das benötigte Image bei
Bedarf automatisch nach — ein separater `make build-*`-Aufruf ist nur nötig,
um das Bauen vom Starten zu trennen (z. B. um vorab zu bauen, ohne den Stack
gleich hochzufahren). Intern ist `make` nur ein dünner Wrapper um
`docker compose --profile <sim|real|real-full>` auf der einen
`docker-compose.yml` — es gibt bewusst keine separaten
`docker-compose.*.yaml`-Dateien mehr.

```bash
# Simulation: MuJoCo-G1 + g1pilot (robot_state, Arme, RViz, Teleop)
G1_SIM_MODE=true docker compose --profile sim up

# Echter Roboter (schlank: Arme + Hände + Unitree-Loco-Controller)
ROBOT_INTERFACE=<iface> docker compose --profile real up

# Echter Roboter mit Livox/MOLA/Navigation (großes Image, MID360 nötig)
ROBOT_INTERFACE=<iface> docker compose --profile real-full up
```

## Nach dem Start: Dokumentation

Alle weiteren Themen-Dokumente sind über [docs/README.md](README.md)
erreichbar, sowie aus dem grafischen Startmenü über den Menüpunkt
„Dokumentation".

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| `docker run --rm hello-world` verlangt `sudo` | Neu einloggen bzw. `newgrp docker` nach Schritt 3. |
| Kein MuJoCo-/RViz-Fenster | `DISPLAY` gesetzt? `xhost +local:docker` gelaufen (macht `make`/`start.sh` automatisch)? |
| Build bricht mit Netzwerkfehlern ab | Docker-Build lädt ROS-2-Pakete aus dem Internet — Firewall/Proxy prüfen. Unter WSL2 mit VPN: [WSL2-Netzwerk](#wsl2-netzwerk-vpn-uni-wlan-roboter). |
| `apt install` hängt bei `Ign:` / `curl` Timeout (WSL2) | WSL kommt nicht am VPN vorbei ins Internet → [WSL2-Netzwerk](#wsl2-netzwerk-vpn-uni-wlan-roboter). |
| `./start.sh` zeigt nur das Text-Menü | `python3-tk` fehlt auf dem Host: `sudo apt install python3-tk`. |
| `setup.sh`: `No matching distribution found for mujoco-scene-editor` / `Requires-Python <3.13` | Host-Python zu neu (Ubuntu 26.04: 3.14) → Python 3.12 per uv, siehe Schritt 5b. |
| `setup.sh`: `No module named 'ensurepip'` | `sudo apt install python3-venv`, dann `rm -rf .venv && ./setup.sh`. |
| Greif-Objekte fliegen weg, ständig `[scene-reset]` | `trimesh`/`vhacdx` fehlen im `scene_editor/.venv` → Schritt 5b. |
| `make sim` baut jedes Mal neu | Normal, sofern sich Quellcode/Dockerfile geändert haben; Docker cached unveränderte Layer. |
| Fenster öffnen sich, aber der Roboter reagiert auf nichts | Zunächst normal — siehe [30_loco_anleitung.md](30_loco_anleitung.md), der Roboter startet bewusst nicht automatisch. |
