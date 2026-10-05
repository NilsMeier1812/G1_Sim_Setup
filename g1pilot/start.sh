#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════════════
#  start.sh — Start-Menue fuer den G1-Stack (Sim UND Real).
#
#  Standard: OHNE Argumente oeffnet sich das grafische Startmenue (g1_gui.py,
#  Tkinter) — Sim/Real/Umgebungen komplett per Klick, kein Terminal noetig.
#  Die GUI ruft am Ende genau dieses Skript mit --yes + Env-Vars auf. Fehlt
#  Tkinter/Display, kommt automatisch das Text-Menue unten. Erzwingen mit
#  --menu (Text) oder G1_NO_GUI=1.
#
#  Text-Menue — allererste Frage: SIMULATION oder ECHTER ROBOTER.
#
#  SIM  : MuJoCo + loco_sim (Whole-Body-Policy). Fragen: RViz, Inspire-
#         Haende, GUI-Auto-Open, Rebuild. Lockstep immer an.
#  REAL : Unitree-Loco-Controller (loco_client) + Arm-Manipulation +
#         Inspire-Haende via Modbus TCP. Fragen: Netzwerk-Interface (mit
#         Auto-Erkennung), Haende + IPs, GUI-Auto-Open, RViz, Rebuild —
#         plus SICHERHEITS-BESTAETIGUNG (der Roboter bewegt sich!).
#
#  Alles hat sinnvolle Defaults — einfach ENTER druecken nimmt den Default.
#  Nicht-interaktiv nutzbar via Env, z.B.:
#    USE_RVIZ=true ./start.sh --yes                       # Sim
#    G1_MODE=real ROBOT_INTERFACE=enp3s0 G1_REAL_CONFIRM=1 ./start.sh --yes
#  (Im --yes-Modus startet REAL nur mit G1_REAL_CONFIRM=1.)
# ════════════════════════════════════════════════════════════════════════
set -e
cd "$(dirname "$0")"

# ── Farben (nur wenn Terminal) ──────────────────────────────────────────
if [ -t 1 ]; then
  B="\033[1m"; DIM="\033[2m"; G="\033[32m"; Y="\033[33m"; C="\033[36m"; R="\033[0m"
else
  B=""; DIM=""; G=""; Y=""; C=""; R=""
fi

ASSUME_YES=0
FORCE_MENU=0
PASSTHRU=()
for a in "$@"; do
  case "$a" in
    -y|--yes) ASSUME_YES=1 ;;        # keine Rueckfragen, Defaults/Env nehmen
    --menu|--no-gui|--tui) FORCE_MENU=1 ;;  # klassisches Text-Menue erzwingen
    *) PASSTHRU+=("$a") ;;           # Rest an `docker compose up` (z.B. --build)
  esac
done

# ── Grafisches Startmenue (Standard) ────────────────────────────────────
#  Ohne Argumente und mit erreichbarem Tkinter startet die GUI (g1_gui.py).
#  Sie sammelt alle Optionen und ruft dann ihrerseits 'start.sh --yes' auf —
#  daher wird die GUI nur im interaktiven Ur-Aufruf gezoegert. Fehlt Tkinter
#  oder ein Display, faellt es lautlos auf das Text-Menue unten zurueck.
#  Erzwingen: --menu (Text) bzw. das Setzen von G1_NO_GUI=1.
if [ "$ASSUME_YES" != "1" ] && [ "$FORCE_MENU" != "1" ] && \
   [ "${G1_NO_GUI:-0}" != "1" ] && [ "${#PASSTHRU[@]}" -eq 0 ]; then
  if command -v python3 >/dev/null 2>&1 && \
     python3 -c 'import tkinter' >/dev/null 2>&1; then
    exec python3 "$(dirname "$0")/g1_gui.py"
  fi
fi

# ── Helfer: Einzelauswahl-Menue. $1=Frage, $2=Default-Index(1-basiert),
#    $3=aktueller Env-Wert (hat im --yes-Modus Vorrang; "" = keiner),
#    danach Paare "Label|WERT". Setzt globale Variable REPLY_VALUE. ───────
ask_menu() {
  local prompt="$1"; shift
  local def="$1"; shift
  local current="$1"; shift
  local labels=() values=()
  while [ "$#" -ge 1 ]; do labels+=("${1%%|*}"); values+=("${1##*|}"); shift; done
  # Nicht-interaktiv: vorgegebener Env-Wert gewinnt, sonst Default-Index.
  if [ "$ASSUME_YES" = "1" ]; then
    if [ -n "$current" ]; then REPLY_VALUE="$current"; else REPLY_VALUE="${values[$((def-1))]}"; fi
    return
  fi
  echo -e "${B}${prompt}${R}"
  local i
  for i in "${!labels[@]}"; do
    local n=$((i+1)); local mark="  "
    [ "$n" = "$def" ] && mark="${G}*${R} "
    echo -e "   ${mark}${C}${n})${R} ${labels[$i]}"
  done
  local choice
  read -rp "$(echo -e "   ${DIM}Auswahl [${def}]:${R} ")" choice
  choice="${choice:-$def}"
  if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "${#values[@]}" ]; then
    echo -e "   ${Y}Ungueltig -> Default ${def}.${R}"; choice="$def"
  fi
  REPLY_VALUE="${values[$((choice-1))]}"
  echo
}

clear 2>/dev/null || true
echo -e "${B}╔══════════════════════════════════════════════╗${R}"
echo -e "${B}║      G1 — Start-Menue (Sim / Real)            ║${R}"
echo -e "${B}╚══════════════════════════════════════════════╝${R}"
echo -e "${DIM}ENTER = markierter Default (*).${R}\n"

# ── 0) Modus: Simulation oder echter Roboter ────────────────────────────
ask_menu "0) Was soll gestartet werden?" 1 "${G1_MODE:-}" \
  "SIMULATION — MuJoCo + Whole-Body-Policy|sim" \
  "ECHTER ROBOTER — G1 per LAN (Unitree-Loco + Arme + Haende)|real"
G1_MODE="$REPLY_VALUE"

# ════════════════════════════════════════════════════════════════════════
#  REAL-ZWEIG
# ════════════════════════════════════════════════════════════════════════
if [ "$G1_MODE" = "real" ]; then
  PROFILE=real

  # ── R1) Netzwerk-Interface zum G1 (Auto-Erkennung) ────────────────────
  #   Kandidaten: physische NICs (kein lo/docker/veth/br-). Das Interface mit
  #   einer 192.168.123.x-Adresse (Unitree-Roboter-LAN) wird als Default markiert.
  _nic_menu=() ; _def_idx=1 ; _i=0
  while read -r _nic; do
    [ -z "$_nic" ] && continue
    case "$_nic" in lo|docker*|veth*|br-*|virbr*) continue ;; esac
    _ip=$(ip -4 -o addr show "$_nic" 2>/dev/null | awk '{print $4}' | head -1)
    _i=$((_i+1))
    _nic_menu+=("${_nic}  ${_ip:-<keine IPv4>}|${_nic}")
    case "$_ip" in 192.168.123.*) _def_idx=$_i ;; esac
  done < <(ip -o link show 2>/dev/null | awk -F': ' '{print $2}' | cut -d@ -f1)
  _nic_menu+=("Anderes Interface (manuell eingeben)|__manual__")

  ask_menu "R1) Netzwerk-Interface zum G1? (Roboter-LAN = 192.168.123.x)" \
    "$_def_idx" "${ROBOT_INTERFACE:-}" "${_nic_menu[@]}"
  if [ "$REPLY_VALUE" = "__manual__" ]; then
    read -rp "$(echo -e "   ${DIM}Interface-Name:${R} ")" REPLY_VALUE
  fi
  export ROBOT_INTERFACE="$REPLY_VALUE"

  # ── R2) Inspire-FTP-Haende (Modbus TCP im Roboter-LAN) ────────────────
  ask_menu "R2) Inspire-FTP-Haende ansteuern? (Modbus TCP, an den G1 angeschlossen)" 1 "${G1_INSPIRE_HANDS:-}" \
    "Ja  — Hand-Bridge mit Modbus-Backend + GUIs|1" \
    "Nein — ohne Haende|0"
  export G1_INSPIRE_HANDS="$REPLY_VALUE"

  if [ "$G1_INSPIRE_HANDS" = "1" ]; then
    if [ "$ASSUME_YES" != "1" ]; then
      read -rp "$(echo -e "   ${DIM}IP linke Hand  [${G1_HAND_LEFT_HOST:-192.168.123.210}]:${R} ")" _l
      read -rp "$(echo -e "   ${DIM}IP rechte Hand [${G1_HAND_RIGHT_HOST:-192.168.123.211}]:${R} ")" _r
      export G1_HAND_LEFT_HOST="${_l:-${G1_HAND_LEFT_HOST:-192.168.123.210}}"
      export G1_HAND_RIGHT_HOST="${_r:-${G1_HAND_RIGHT_HOST:-192.168.123.211}}"
      echo
    else
      export G1_HAND_LEFT_HOST="${G1_HAND_LEFT_HOST:-192.168.123.210}"
      export G1_HAND_RIGHT_HOST="${G1_HAND_RIGHT_HOST:-192.168.123.211}"
    fi
    ask_menu "R2b) Hand-GUIs automatisch im Browser oeffnen (und verbinden)?" 1 "${OPEN_GUIS:-}" \
      "Ja  — Controller + Viewer oeffnen sich selbst und verbinden|true" \
      "Nein — GUIs manuell oeffnen (http://localhost:8767/...)|false"
    export OPEN_GUIS="$REPLY_VALUE"
  else
    export OPEN_GUIS=false
  fi

  # ── R3) RViz ──────────────────────────────────────────────────────────
  ask_menu "R3) RViz mitstarten? (rviz-Marker = Interface der Arm-Manipulation)" 1 "${USE_RVIZ:-}" \
    "Ja  — RViz an (empfohlen: IK-Marker + Zustand sichtbar)|true" \
    "Nein — ohne RViz (nur Streamdeck/PS4)|false"
  export USE_RVIZ="$REPLY_VALUE"

  # ── R4) Rebuild ───────────────────────────────────────────────────────
  ask_menu "R4) Docker-Images vor dem Start neu bauen?" 1 "" \
    "Nein — vorhandene Images nutzen (schnell)|0" \
    "Ja  — --build (nach Code-/Dockerfile-Aenderungen)|1"
  if [ "$REPLY_VALUE" = "1" ]; then PASSTHRU+=("--build"); fi

  export G1_SIM_MODE=false

  # ── Zusammenfassung + SICHERHEITS-GATE ────────────────────────────────
  echo -e "${B}\033[31m╔══════════════════════════════════════════════════════════╗${R}"
  echo -e "${B}\033[31m║  ACHTUNG: ECHTER ROBOTER — der G1 wird sich bewegen!      ║${R}"
  echo -e "${B}\033[31m╚══════════════════════════════════════════════════════════╝${R}"
  echo -e "   Interface      : ${G}ROBOT_INTERFACE=${ROBOT_INTERFACE}${R}"
  _hands_lbl=$( [ "${G1_INSPIRE_HANDS}" = "1" ] && echo "Modbus ${G1_HAND_LEFT_HOST} / ${G1_HAND_RIGHT_HOST}" || echo "aus" )
  echo -e "   Haende         : ${G}G1_INSPIRE_HANDS=${G1_INSPIRE_HANDS}${R} ${DIM}(${_hands_lbl})${R}"
  echo -e "   RViz           : ${G}USE_RVIZ=${USE_RVIZ}${R}"
  echo -e "   Walk-Limits    : ${G}vx=${G1_MAX_VX:-0.4} vy=${G1_MAX_VY:-0.3} vyaw=${G1_MAX_VYAW:-0.4}${R}"
  echo -e "   ${DIM}Kein Auto-Start: Der Roboter bewegt sich erst nach STREAMDECK-${R}"
  echo -e "   ${DIM}Kommandos (START -> START BALANCING -> ...).${R}"
  echo -e "   ${Y}E-STOP (Streamdeck) = Damp = Roboter sackt ZUSAMMEN (sichern!).${R}"
  echo -e "   ${DIM}Checkliste vor dem ersten Lauf: g1pilot/docs/70_echtroboter_anleitung.md${R}"
  echo
  if [ "$ASSUME_YES" = "1" ]; then
    if [ "${G1_REAL_CONFIRM:-0}" != "1" ]; then
      echo -e "${Y}[start] REAL im --yes-Modus braucht G1_REAL_CONFIRM=1 — Abbruch.${R}"
      exit 1
    fi
  else
    read -rp "$(echo -e "   ${B}Zum Bestaetigen 'REAL' tippen (alles andere bricht ab):${R} ")" _confirm
    if [ "$_confirm" != "REAL" ]; then
      echo -e "${Y}[start] Abgebrochen — nichts gestartet.${R}"
      exit 1
    fi
  fi
  echo

# ════════════════════════════════════════════════════════════════════════
#  SIM-ZWEIG (bisheriger Ablauf)
# ════════════════════════════════════════════════════════════════════════
else
  PROFILE=sim

  # ── 1) RViz ───────────────────────────────────────────────────────────
  ask_menu "1) RViz mitstarten? (MuJoCo-Fenster kommt immer)" 2 "${USE_RVIZ:-}" \
    "Ja  — RViz an (CoM-/TF-Visualisierung; dank Lockstep gefahrlos)|true" \
    "Nein — nur MuJoCo-Fenster|false"
  export USE_RVIZ="$REPLY_VALUE"

  # ── 2) Inspire-FTP-Haende ─────────────────────────────────────────────
  #   Master-Schalter: waehlt im MuJoCo das Inspire-Finger-Modell UND startet die
  #   Hand-Bridge (HTML-GUIs :8766/:8765 + DDS, Finger steuerbar/gemessen in RViz).
  ask_menu "2) Inspire-FTP-Haende? (Finger steuerbar + GUIs; sonst Rubber-Hand)" 2 "${G1_INSPIRE_HANDS:-}" \
    "Ja  — Inspire-Modell + Hand-Bridge (GUIs :8766/:8765, echte Kraefte)|1" \
    "Nein — bisheriges Rubber-Hand-Modell|0"
  export G1_INSPIRE_HANDS="$REPLY_VALUE"

  # ── 2b) Hand-GUIs automatisch im Browser oeffnen (nur mit Inspire-Haenden) ─
  if [ "$G1_INSPIRE_HANDS" = "1" ]; then
    ask_menu "2b) Hand-GUIs automatisch im Browser oeffnen (und verbinden)?" 1 "${OPEN_GUIS:-}" \
      "Ja  — Controller + Viewer oeffnen sich selbst und verbinden|true" \
      "Nein — GUIs manuell oeffnen (web/*.html)|false"
    export OPEN_GUIS="$REPLY_VALUE"
  else
    export OPEN_GUIS=false
  fi

  # ── 2d) Umgebung waehlen (G1 bleibt gleich, nur die Welt drumherum) ────
  #   Umgebungen werden im scene_editor gebaut: je Umgebung ein Ordner
  #   scene_editor/scenes/<name>/ (umgebung.xml + meshes/, siehe env_store.py).
  #   "Standard" = bisheriges Terrain (scene.xml). Bei einer Auswahl erzeugt
  #   build_env_scene.py auf dem HOST eine kombinierte Szene (G1 + Umgebung) im
  #   g1-Ordner; der Container mountet das Repo nur read-only, kann also selbst
  #   nichts schreiben.
  _scene_editor="../unitree_mujoco/scene_editor"
  _scenes_dir="$_scene_editor/scenes"
  # Python fuer den Szenen-Generator: das Editor-venv hat mujoco und prueft die
  # kombinierte Szene damit gleich mit; sonst der System-Python (ohne Pruefung).
  _scene_py="python3"
  [ -x "$_scene_editor/.venv/bin/python" ] && _scene_py="$_scene_editor/.venv/bin/python"

  _env_menu=("Standard — aktuelles Terrain (scene.xml)|__default__")
  # Liste aus env_store.py: nur Namen, die als G1_ENV taugen (Leer-/Sonder-
  # zeichen brechen beim docker-compose-Durchreichen); Beifang aelterer
  # Editor-Exporte und kaputte Dateien sortiert es aus.
  while IFS= read -r _b; do
    [ -n "$_b" ] && _env_menu+=("$_b|$_b")
  done < <("$_scene_py" "$_scene_editor/env_store.py" list 2>/dev/null || true)
  ask_menu "2d) Welche Umgebung laden? (G1 wird unveraendert hineingeladen)" 1 "${G1_ENV:-}" "${_env_menu[@]}"
  if [ "$REPLY_VALUE" = "__default__" ] || [ -z "$REPLY_VALUE" ]; then
    G1_ENV=""
  else
    G1_ENV="$REPLY_VALUE"
    if ! "$_scene_py" "$_scene_editor/env_store.py" resolve "$G1_ENV" >/dev/null 2>&1; then
      echo -e "${Y}[start] Umgebung '${G1_ENV}' gibt es nicht (mehr) in ${_scenes_dir}/ -> Standard.${R}"
      G1_ENV=""
    elif ! _env_out=$("$_scene_py" "$_scene_editor/build_env_scene.py" \
           --env "$G1_ENV" --inspire "${G1_INSPIRE_HANDS:-0}" 2>&1 >/dev/null); then
      echo -e "${Y}[start] Umgebung '${G1_ENV}' konnte nicht geladen werden -> Standard.${R}"
      [ -n "$_env_out" ] && echo -e "${DIM}${_env_out}${R}"
      echo -e "${DIM}        Reparieren: Menue 'Umgebungen bearbeiten' -> 'Auf Ladbarkeit pruefen'.${R}"
      G1_ENV=""
    elif [ -n "$_env_out" ]; then
      # Warnungen des Generators (fehlende Meshes o.ae.) sichtbar machen.
      echo -e "${DIM}${_env_out}${R}"
    fi
  fi
  export G1_ENV

  # ── 2c) Navigation (g1pilot-Ansatz) mitstarten ───────────────────────
  ask_menu "2c) Navigation mitstarten? (dijkstra_planner + nav2point + Sim-Glue)" 2 "${G1_ENABLE_NAV:-}" \
    "Ja  — Nav-Stack an (Ziel per RViz/CLI; siehe g1pilot/docs/50_navigation_anleitung.md)|1" \
    "Nein — ohne Navigation (nur Teleop/Loco)|0"
  export G1_ENABLE_NAV="$REPLY_VALUE"

  # ── 3) Rebuild ────────────────────────────────────────────────────────
  ask_menu "3) Docker-Images vor dem Start neu bauen?" 1 "" \
    "Nein — vorhandene Images nutzen (schnell)|0" \
    "Ja  — --build (nach Code-/Dockerfile-Aenderungen)|1"
  if [ "$REPLY_VALUE" = "1" ]; then PASSTHRU+=("--build"); fi

  # ── Feste Sim-Voreinstellungen (Loco-Modus, Basis frei) ───────────────
  export HOLD_BASE_MODE=off            # Basis frei -> die Policy regelt den Koerper
  # Lockstep ist im Betrieb IMMER an (deterministische 50-Hz-Regelrate, PC-unabhaengig).
  export SIM_LOCKSTEP=1
  # Eine velocity-konditionierte Whole-Body-Policy macht Stehen UND Laufen: cmd=0
  # -> stehen, cmd!=0 -> laufen. Kein separater Balance-Regler mehr zu waehlen.
  # Lockstep ist auf Echtzeit gedeckelt; SIM_REALTIME_FACTOR steuert das Tempo (1.0=Echtzeit).
  export SIM_REALTIME_FACTOR=${SIM_REALTIME_FACTOR:-1.0}
  export G1_SIM_MODE=true

  # ── Zusammenfassung ───────────────────────────────────────────────────
  echo -e "${B}Starte mit:${R}"
  _env_lbl=$( [ -n "${G1_ENV}" ] && echo "${G1_ENV} (scene_env_${G1_ENV}.xml)" || echo "Standard (scene.xml)" )
  echo -e "   Umgebung       : ${G}G1_ENV=${G1_ENV:-<default>}${R} ${DIM}(${_env_lbl})${R}"
  echo -e "   RViz           : ${G}USE_RVIZ=${USE_RVIZ}${R}"
  _hands_lbl=$( [ "${G1_INSPIRE_HANDS}" = "1" ] && echo "Inspire-FTP (Finger + GUIs)" || echo "Rubber-Hand" )
  echo -e "   Haende         : ${G}G1_INSPIRE_HANDS=${G1_INSPIRE_HANDS}${R} ${DIM}(${_hands_lbl})${R}"
  [ "${G1_INSPIRE_HANDS}" = "1" ] && echo -e "   Hand-GUIs      : ${G}OPEN_GUIS=${OPEN_GUIS}${R} ${DIM}(Browser-Auto-Open :8766/:8765)${R}"
  echo -e "   Lockstep       : ${G}SIM_LOCKSTEP=${SIM_LOCKSTEP}${R} ${DIM}(immer an, auf Echtzeit gedeckelt)${R}"
  echo -e "   Basis          : ${G}HOLD_BASE_MODE=${HOLD_BASE_MODE}${R} (Loco-Modus)"
  echo -e "   Loco-Policy    : ${G}g1_wholebody${R} ${DIM}(Stehen=cmd0; Laufen via Streamdeck)${R}"
  _nav_lbl=$( [ "${G1_ENABLE_NAV}" = "1" ] && echo "an (dijkstra + nav2point + Sim-Glue)" || echo "aus" )
  echo -e "   Navigation     : ${G}G1_ENABLE_NAV=${G1_ENABLE_NAV}${R} ${DIM}(${_nav_lbl})${R}"
  [ "${#PASSTHRU[@]}" -gt 0 ] && echo -e "   compose-Args   : ${G}${PASSTHRU[*]}${R}"
  echo
fi

# ── X11 freigeben (MuJoCo-Viewer + ggf. RViz brauchen den X-Server) ─────
xhost +local:root >/dev/null 2>&1 || \
  echo -e "${Y}[start] WARN: 'xhost' nicht verfuegbar - laeuft hier ein X-Server?${R}"

# ── Hand-GUIs oeffnen: Host-Watcher ─────────────────────────────────────
#  start.sh laeuft auf dem HOST (nicht im Container) — nur HIER existiert ein
#  Browser. Der Container (ui_interface/Streamdeck-Button) kann selbst keinen
#  Browser starten; er fasst stattdessen die Trigger-Datei .gui_open_request im
#  bind-gemounteten Repo an. Dieser Watcher:
#    1) wartet, bis die Controller-Bridge auf :8766 lauscht (colcon-Build dauert),
#    2) oeffnet die GUIs einmal automatisch (nur wenn OPEN_GUIS=true),
#    3) bleibt dann am Leben und oeffnet sie erneut, sobald die Trigger-Datei
#       ihre mtime aendert -> der Streamdeck-Button "INSPIRE FTP GUIs" wirkt.
#  Beide GUIs werden mit ?autoconnect=1 geoeffnet -> verbinden sich von selbst.
gui_opener_daemon() {
  # Die GUIs werden von der Bridge per HTTP serviert (Port 8767). file://-URLs
  # mit ?autoconnect=1 scheiterten je nach System (xdg-open/WSL behandeln den
  # Query-String als Teil des Dateinamens) -> Seite ging nicht auf bzw. verband
  # sich nicht. http-URLs funktionieren mit JEDEM Opener.
  local ctrl="http://localhost:8767/hand_controller_viewer.html?autoconnect=1"
  local view="http://localhost:8767/inspire_hand_viewer.html?autoconnect=1"
  local trigger="$PWD/.gui_open_request"   # == /ros2_ws/src/g1pilot/.gui_open_request im Container

  # WSL2-Erkennung (Kernel-Release enthaelt "microsoft")
  local is_wsl=0
  grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null && is_wsl=1

  # Oeffnet eine URL im System-Browser; gibt 0 zurueck wenn erfolgreich.
  _open_url() {
    local url="$1"
    if [ "$is_wsl" = "1" ]; then
      # WSL2: Browser laeuft auf Windows; http-URLs kann jeder Weg direkt oeffnen.
      if command -v wslview >/dev/null 2>&1; then
        wslview "$url" >/dev/null 2>&1; return 0
      fi
      if command -v cmd.exe >/dev/null 2>&1; then
        cmd.exe /c start "" "$url" >/dev/null 2>&1; return 0
      fi
      command -v explorer.exe >/dev/null 2>&1 && { explorer.exe "$url" >/dev/null 2>&1; return 0; }
    else
      local c
      for c in xdg-open open sensible-browser x-www-browser \
               microsoft-edge microsoft-edge-stable msedge \
               firefox google-chrome chromium chromium-browser; do
        command -v "$c" >/dev/null 2>&1 && { "$c" "$url" >/dev/null 2>&1; return 0; }
      done
    fi
    return 1
  }

  # Beide GUIs nacheinander oeffnen; bei fehlendem Opener Pfade ausgeben.
  _open_both() {
    if ! _open_url "$ctrl"; then
      echo -e "${Y}[start] Kein Browser-Opener gefunden - GUIs bitte manuell oeffnen:${R}"
      echo -e "   ${C}$ctrl${R}"
      echo -e "   ${C}$view${R}"
      return 1
    fi
    sleep 1
    _open_url "$view" >/dev/null 2>&1 || true
  }

  # Port-Check (bash /dev/tcp) — true, solange die Bridge auf :8766 lauscht.
  _bridge_up() { (exec 3<>"/dev/tcp/127.0.0.1/8766") 2>/dev/null; }

  # 1) Warten bis die Bridge lauscht (max ~5 min, deckt auch den ersten Build ab).
  local i=0
  while [ $i -lt 600 ]; do _bridge_up && break; sleep 0.5; i=$((i+1)); done

  # Stale-Trigger aus einem frueheren Lauf entfernen, damit der Watcher nicht
  # sofort faelschlich oeffnet.
  rm -f "$trigger" 2>/dev/null || true

  # 2) Einmaliges Auto-Open (nur wenn gewuenscht).
  [ "$OPEN_GUIS" = "true" ] && _open_both

  # 3) Watcher-Schleife: Trigger-Datei (vom Streamdeck-Button angefasst) beobachten.
  #    Endet, sobald die Bridge nicht mehr lauscht (Sim gestoppt) -> kein Leak.
  local last=""
  while _bridge_up; do
    if [ -f "$trigger" ]; then
      local cur
      cur=$(stat -c %Y "$trigger" 2>/dev/null || stat -f %m "$trigger" 2>/dev/null || echo "")
      if [ -n "$cur" ] && [ "$cur" != "$last" ]; then
        last="$cur"
        echo -e "${C}[start] Streamdeck: oeffne Inspire-FTP-GUIs...${R}"
        _open_both
      fi
    fi
    sleep 0.5
  done
}

# Watcher immer starten, wenn Inspire-Haende aktiv sind: er bedient das optionale
# Auto-Open UND den Streamdeck-Button (auch bei OPEN_GUIS=false). Gilt fuer Sim
# UND Real gleichermassen — die Bridge serviert die GUIs in beiden Modi identisch.
if [ "$G1_INSPIRE_HANDS" = "1" ]; then
  ( gui_opener_daemon ) &
fi

# ── GPU fuer MuJoCo-Viewer/RViz ─────────────────────────────────────────
# Sonst rendern die Container auf der CPU (llvmpipe) -> CPU fehlt Sim/Regler.
# NVIDIA (nur mit Container Toolkit) bevorzugt, sonst /dev/dri (Intel/AMD).
# Siehe docker-compose.nvidia.yml / docker-compose.gpu.yml.
if [ -z "${COMPOSE_FILE:-}" ]; then
  _cf="docker-compose.yml"
  [ -e /dev/dri ] && _cf="$_cf:docker-compose.gpu.yml"
  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1 \
     && command -v nvidia-container-runtime-hook >/dev/null 2>&1; then
    _cf="$_cf:docker-compose.nvidia.yml"
  fi
  export COMPOSE_FILE="$_cf"
  echo -e "${G}[start] Compose-Dateien: ${COMPOSE_FILE}${R}"
fi
# PRIME Render Offload nur auf Hybrid-Systemen (Bildschirm an Intel/AMD).
# Treibt die NVIDIA selbst den Bildschirm, bleibt das MuJoCo-Fenster mit
# Offload schwarz. Siehe docker-compose.nvidia.yml.
if [ -z "${G1_NV_PRIME_OFFLOAD:-}" ]; then
  if nvidia-smi --query-gpu=display_active --format=csv,noheader 2>/dev/null \
       | grep -q Enabled; then
    export G1_NV_PRIME_OFFLOAD=0
  else
    export G1_NV_PRIME_OFFLOAD=1
  fi
fi

# ── Reste eines frueheren Laufs sauber entfernen ────────────────────────
docker compose --profile "$PROFILE" down --remove-orphans

# ── Hochfahren ──────────────────────────────────────────────────────────
echo -e "${G}[start] docker compose --profile ${PROFILE} up ${PASSTHRU[*]}${R}"
exec docker compose --profile "$PROFILE" up "${PASSTHRU[@]}"
