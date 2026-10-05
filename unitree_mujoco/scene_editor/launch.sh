#!/usr/bin/env bash
# =====================================================================
# Starter fuer den mujoco-scene-editor und den MuJoCo-Viewer.
#
# Umgebungen:  scene_editor/scenes/<name>/umgebung.xml  (+ meshes/ daneben)
#   -> Jede Umgebung ist EIN Ordner mit allem, was sie braucht - kopieren,
#      zippen, loeschen, fertig. Dieselben Umgebungen sind hier UND beim
#      G1-Start (g1pilot/start.sh) waehlbar. Neue einfach im Editor speichern
#      (dort wird nur der NAME gefragt, siehe run_editor.py).
#
# Ohne Argument -> interaktives Menue: listet alle Umgebungen nummeriert auf;
# pro Umgebung kannst du: bearbeiten (Editor), ansehen, mit dem G1 ansehen,
# als Zip packen.
#
#   ./launch.sh                   interaktives Menue (empfohlen)
#   ./launch.sh new               leere Umgebung im Editor starten
#   ./launch.sh edit <name>       bestimmte Umgebung im Editor oeffnen
#   ./launch.sh prompt "text"     Umgebung per Text-Prompt generieren (API-Key)
#   ./launch.sh view <name>       Umgebung allein im MuJoCo-Viewer ansehen
#   ./launch.sh with-g1 <name>    Umgebung + G1 im MuJoCo-Viewer ansehen
#   ./launch.sh list              vorhandene Umgebungen auflisten
#   ./launch.sh view-g1           statisches Beispiel scene_g1_playground.xml
#   ./launch.sh import <datei>    CAD/Mesh (STEP, IGES, STL, OBJ, PLY, GLB, ...) als
#                                 Umgebung importieren -> scenes/<name>/
#                                 (Optionen: ./launch.sh import --help)
#   ./launch.sh pack <name> [ziel]  Umgebung als Zip (Default: export/<name>.zip)
#   ./launch.sh unpack <zip> [--name N] [--force]   Zip als Umgebung einspielen
#   ./launch.sh check [name]      Umgebung(en) auf Vollstaendigkeit pruefen
#   ./launch.sh migrate           alte Umgebungen (scenes/<name>.xml) umwandeln
#   ./launch.sh check-meshes      STLs auf MuJoCo-Tauglichkeit pruefen
#
# <name> darf "kueche", "scenes/kueche" oder der Pfad zur umgebung.xml sein.
# Dateipfade (import/pack/unpack) duerfen relativ zum aktuellen Ordner sein.
#
# Der Editor oeffnet einen lokalen Webserver (http://127.0.0.1:8080; anderer
# Port via SCENE_EDITOR_PORT=8081).
# =====================================================================
set -euo pipefail
CALLER_PWD="$PWD"                 # fuer relative Dateipfade der Nutzer
cd "$(dirname "$0")"
SE_DIR="$PWD"

VENV="${SCENE_EDITOR_VENV:-.venv}"   # Docker-Image: /opt/scene_editor_venv
case "$VENV" in /*) ;; *) VENV="$SE_DIR/$VENV" ;; esac
if [[ ! -x "$VENV/bin/python" ]]; then
  echo "Kein virtualenv gefunden. Bitte zuerst  ./setup.sh  ausfuehren." >&2
  exit 1
fi
# Ein .venv aus einem abgebrochenen setup.sh (z.B. ohne python3-venv) hat
# python, aber keine Pakete -> sonst kryptischer ImportError (cachier o.ae.).
if ! "$VENV/bin/python" -c "import mujoco_scene_editor, cachier, trimesh" 2>/dev/null; then
  echo "virtualenv ist unvollstaendig (Pakete fehlen). Neu aufsetzen mit:" >&2
  echo "  rm -rf \"$VENV\" && ./setup.sh" >&2
  exit 1
fi

# RoBits-Config persistent halten
export ROBITS_CONFIG_DIR="${ROBITS_CONFIG_DIR:-$SE_DIR/.robits_config}"
export SCENE_EDITOR_PORT="${SCENE_EDITOR_PORT:-8080}"

PY="$VENV/bin/python"
G1_PLAYGROUND="../unitree_robots/g1/scene_g1_playground.xml"
EXPORT_DIR="$SE_DIR/export"
mkdir -p scenes

# --- Selbstheilung: fehlende Teile im venv nachinstallieren ------------
# Aeltere venvs haben die Pakete fuer den CAD-Import nicht (cadquery-ocp =
# OpenCascade fuer STEP/IGES, coacd = konvexe Zerlegung konkaver Teile). Statt
# den Nutzer im Editor mit "kein Backend installiert" stehen zu lassen, holen
# wir sie hier einmalig nach. Schlaegt das fehl (kein Internet), laeuft der
# Editor trotzdem - nur eben ohne STEP bzw. mit groeberer Kollision.
ensure_cad_backend() {
  # vhacdx: Kollisions-Zerlegung statischer STL-Hindernisse (build_env_scene.py).
  # Venvs von vor dem CAD-Import haben es, neuere evtl. nicht - leise nachholen.
  if ! "$PY" -c "import vhacdx" >/dev/null 2>&1; then
    "$VENV/bin/pip" install -q vhacdx >/dev/null 2>&1 || true
  fi
  if "$PY" cad_import.py --check >/dev/null 2>&1; then
    return 0
  fi
  echo ">> CAD-Import fehlt im venv - installiere cadquery-ocp + coacd nach"
  echo "   (einmalig, ~80 MB; danach nie wieder)."
  "$VENV/bin/pip" install cadquery-ocp coacd trimesh || true
  if "$PY" cad_import.py --check >/dev/null 2>&1; then
    echo ">> CAD-Import ist jetzt aktiv."
  else
    echo ">> WARNUNG: Nachinstallation fehlgeschlagen (kein Internet?)." >&2
    echo "   Der Editor startet trotzdem, kann aber nur STL/OBJ - keine STEP." >&2
  fi
}

# --- Umgebungen im alten Format (scenes/<name>.xml) einmalig umwandeln --
# Ab jetzt ist jede Umgebung ein Ordner. Alte Dateien werden beim ersten Start
# automatisch umgezogen (Meshes werden in den Ordner KOPIERT, die Bibliothek
# meshes/ bleibt, wie sie ist).
auto_migrate() {
  "$PY" env_store.py migrate --quiet || true
}

# --- Umgebungen (alles ueber env_store.py - EINE Stelle kennt das Format) --
collect_envs() {
  ENVS=()
  local n
  while IFS= read -r n; do
    [[ -n "$n" ]] && ENVS+=("$n")
  done < <("$PY" env_store.py list)
}

# Name/Pfad -> Pfad der umgebung.xml (mit Liste der Umgebungen bei Tippfehler).
resolve_env() {
  local arg="$1"
  if "$PY" env_store.py resolve "$arg" 2>/dev/null; then
    return 0
  fi
  echo "Umgebung nicht gefunden: $arg" >&2
  collect_envs
  if [[ ${#ENVS[@]} -eq 0 ]]; then
    echo "In scenes/ liegt noch keine Umgebung - mit './launch.sh new' eine anlegen." >&2
  else
    echo "Vorhanden in scenes/:" >&2
    local e
    for e in "${ENVS[@]}"; do echo "   - $e" >&2; done
  fi
  return 1
}

edit_scene() { ensure_cad_backend; exec "$PY" run_editor.py edit "$1"; }
new_scene()  { ensure_cad_backend; exec "$PY" run_editor.py new; }

# Werkzeuge, die Dateipfade des Nutzers bekommen, im Aufruf-Ordner starten,
# damit relative Pfade stimmen (die Skripte finden ihre Ordner selbst).
in_caller_dir() { cd "$CALLER_PWD"; exec "$PY" "$@"; }

# CAD-/Mesh-Datei direkt als Umgebung importieren (ohne Editor).
import_file() { ensure_cad_backend; in_caller_dir "$SE_DIR/cad_import.py" "$@"; }

pack_env() {
  local name="$1" target="${2:-}"
  if [[ -z "$target" ]]; then
    mkdir -p "$EXPORT_DIR"
    target="$EXPORT_DIR"
  fi
  in_caller_dir "$SE_DIR/env_store.py" pack "$name" "$target"
}

# Der MuJoCo-Viewer ist ein Desktop-Fenster - im Docker-Container (Editor-
# Profil) gibt es keins. Dann klar sagen statt mit einem GLFW-Fehler abzubrechen.
need_display() {
  if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    echo "Kein Bildschirm verfuegbar (z.B. im Docker-Container) - der MuJoCo-Viewer" >&2
    echo "braucht ein Desktop-Fenster. Im Container geht der Editor im Browser;" >&2
    echo "die Umgebung mit dem G1 dann ueber den Sim-Start (g1pilot/start.sh) ansehen." >&2
    exit 1
  fi
}

view_scene() { need_display; exec "$PY" -m mujoco.viewer --mjcf="$1"; }

# Umgebung + G1 kombinieren und im Viewer ansehen (ohne Docker-Stack).
view_with_g1() {
  local env="$1"
  local out
  if ! out=$("$PY" build_env_scene.py --env "$env" --inspire "${G1_INSPIRE_HANDS:-0}"); then
    echo "Konnte kombinierte Szene nicht erzeugen (Meldung oben)." >&2; exit 1
  fi
  echo "Kombiniert: $out"
  need_display       # Szene ist gebaut + geprueft; nur das Fenster fehlt ggf.
  exec "$PY" -m mujoco.viewer --mjcf="$out"
}

list_envs() {
  collect_envs
  if [[ ${#ENVS[@]} -eq 0 ]]; then
    echo "(noch keine Umgebungen in scenes/)"
    return 0
  fi
  printf '%s\n' "${ENVS[@]}"
}

# Pfad-Eingabe aus dem Menue: Anfuehrungszeichen weg (Drag & Drop ins
# Terminal setzt sie oft).
read_path() {
  local prompt="$1" p sq="'"
  read -rp "$prompt" p
  p="${p%\"}"; p="${p#\"}"; p="${p%$sq}"; p="${p#$sq}"
  printf '%s' "$p"
}

# --- interaktives Menue ----------------------------------------------
menu() {
  collect_envs
  echo ""
  echo "=================== MuJoCo Scene Editor ==================="
  echo "Umgebungen in scene_editor/scenes/"
  echo "(dieselben, die auch beim G1-Start via g1pilot/start.sh waehlbar sind)"
  echo "----------------------------------------------------------"
  if [[ ${#ENVS[@]} -eq 0 ]]; then
    echo "   (noch keine Umgebungen - waehle 'n' fuer eine neue)"
  else
    local i
    for i in "${!ENVS[@]}"; do
      printf "   %2d) %s\n" "$((i + 1))" "${ENVS[$i]}"
    done
  fi
  echo "    n) neue leere Umgebung im Editor"
  echo "    i) CAD-/Mesh-Datei (STEP, STL, ...) als Umgebung importieren"
  echo "    u) Umgebung aus Zip einspielen"
  echo "    q) beenden"
  echo "----------------------------------------------------------"
  local sel file
  read -rp "Auswahl (Zahl / n / i / u / q): " sel

  case "$sel" in
    q|Q|"") exit 0 ;;
    n|N)    new_scene ;;
    i|I)
      file=$(read_path "Pfad zur Datei: ")
      [[ -n "$file" ]] || exit 0
      import_file "$file"
      ;;
    u|U)
      file=$(read_path "Pfad zum Zip: ")
      [[ -n "$file" ]] || exit 0
      in_caller_dir "$SE_DIR/env_store.py" unpack "$file"
      ;;
    *[!0-9]*) echo "Ungueltige Eingabe." >&2; exit 1 ;;
    *)
      local idx=$((sel - 1))
      if (( idx < 0 || idx >= ${#ENVS[@]} )); then
        echo "Ungueltige Nummer." >&2; exit 1
      fi
      local chosen="${ENVS[$idx]}" xml act
      xml=$(resolve_env "$chosen") || exit 1
      echo ""
      echo "Gewaehlt: $chosen"
      echo "Aktion:"
      echo "   e) im Editor bearbeiten"
      echo "   v) allein im Viewer ansehen (ohne Roboter)"
      echo "   g) mit dem G1 im Viewer ansehen"
      echo "   p) als Zip packen (-> export/$chosen.zip)"
      read -rp "Auswahl [e/v/g/p] (Default e): " act
      act="${act:-e}"
      case "$act" in
        e|E) edit_scene "$xml" ;;
        v|V) view_scene "$xml" ;;
        g|G) view_with_g1 "$xml" ;;
        p|P) pack_env "$chosen" ;;
        *)   echo "Ungueltige Aktion." >&2; exit 1 ;;
      esac
      ;;
  esac
}

# --- Dispatch --------------------------------------------------------
CMD="${1:-menu}"; shift || true

case "$CMD" in
  -h|--help|help|check-meshes) ;;
  *) auto_migrate ;;
esac

# Fuer die Kommandos mit Umgebungs-Argument: Default = Starter-Umgebung, und
# der Name wird nachgeschlagen statt blind durchgereicht.
need_env() {
  resolve_env "${1:-environment_starter}"
}

need_arg() {
  if [[ -z "${1:-}" ]]; then
    echo "$2" >&2
    exit 1
  fi
}

case "$CMD" in
  menu)    menu ;;
  list)    list_envs ;;
  new)     new_scene ;;
  edit)    ENV_FILE=$(need_env "${1:-}") || exit 1; edit_scene "$ENV_FILE" ;;
  prompt)
    need_arg "${1:-}" "Bitte eine Beschreibung angeben, z.B.:
   ./launch.sh prompt \"a kitchen with a table and two boxes\""
    ensure_cad_backend
    exec "$PY" run_editor.py prompt "$@"
    ;;
  view)    ENV_FILE=$(need_env "${1:-}") || exit 1; view_scene "$ENV_FILE" ;;
  with-g1) ENV_FILE=$(need_env "${1:-}") || exit 1; view_with_g1 "$ENV_FILE" ;;
  view-g1) view_scene "$G1_PLAYGROUND" ;;
  import|convert)
    need_arg "${1:-}" "Bitte eine Datei angeben, z.B.:
   ./launch.sh import ~/Downloads/zelle.stp
   ./launch.sh import --help     (alle Optionen)"
    import_file "$@"
    ;;
  pack)
    need_arg "${1:-}" "Welche Umgebung? z.B.:  ./launch.sh pack kueche [ziel.zip|ordner]"
    pack_env "$@"
    ;;
  unpack)
    need_arg "${1:-}" "Welches Zip? z.B.:  ./launch.sh unpack ~/Downloads/kueche.zip [--name N] [--force]"
    in_caller_dir "$SE_DIR/env_store.py" unpack "$@"
    ;;
  check)   exec "$PY" env_store.py check "$@" ;;
  migrate) exec "$PY" env_store.py migrate ;;
  check-meshes) in_caller_dir "$SE_DIR/mesh_utils.py" "$@" ;;
  -h|--help|help)
    sed -n '2,38p' "$0" | sed 's/^# \{0,1\}//'
    ;;
  *)
    echo "Unbekanntes Kommando: $CMD" >&2
    echo "Benutze: (ohne Argument) | new | edit [name] | prompt \"text\" | view [name] | with-g1 [name] | list | view-g1 | import <datei> | pack <name> | unpack <zip> | check [name] | migrate | check-meshes" >&2
    exit 1
    ;;
esac
